/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * lte_sumo_ho_dataset_extractor_50ues_two_triggers_paperA3_paperKPI.cc
 *
 * CSV columns (15):
 * time_s,ue_imsi,rssi_id,rssi,current_id,current_rssi,hysteresis,
 * rsrp,sinr_db_reported,sinr_linear,sinr_db_calc,
 * distance_ue,trigger_a3,trigger_kpi,gap_db
 *
 * Definitions:
 * - rsrp (col8): serving RSRP (dBm) from MeasurementReport encoding (-140 + rsrpResult)
 * - rssi_id/rssi (col3/4): best neighbor candidate physCellId + its RSRP (dBm) from MeasurementReport
 * - current_rssi (col6): set to serving RSRP (dBm) for stability (paper-style)
 * - gap_db (col15): (candidate_rsrp - serving_rsrp) in dB
 * - sinr_linear (col10): raw SINR (linear) from UE PHY ReportCurrentCellRsrpSinr
 * - sinr_db_calc (col11): 10*log10(sinr_linear)
 * - sinr_db_reported (col9): same as sinr_db_calc (kept for compatibility)
 *
 * trigger_a3 (col13): Event A3 style pulse:
 *   condition: candidate_rsrp > serving_rsrp + hysteresis
 *   persistence: must hold for TTT=160ms, emit one pulse per episode
 *
 * trigger_kpi (col14): Paper-style KPI pulse (RSRP/SINR/distance):
 *   condition: (rsrp < thr_rsrp) OR (sinr_db_calc < thr_sinr) OR (distance_ue > thr_dist)
 *   persistence: 2-sample confirmation at dataset sampling resolution (1s), rising-edge pulse
 *
 * Validity:
 * - never emit triggers when current_id==0 or distance_ue is NaN
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/ipv4-static-routing-helper.h"

#include <fstream>
#include <unordered_map>
#include <iomanip>
#include <cmath>
#include <limits>
#include <regex>
#include <cstdint>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteSumoHoDatasetExtractor");

// ---- Paper-style KPI thresholds (tune as needed) ----
// Based on your observed ranges, these will actually fire sometimes.
static const double KPI_RSRP_THRESH_DBM = -80.0; // serving RSRP below this => bad
static const double KPI_SINR_THRESH_DB  =  45.0; // your SINR scale is high; tune if needed
static const double KPI_DIST_THRESH_M   =  770.0; // UE farther than this from serving => risk
// ----------------------------------------------------

struct PerUeState
{
  uint16_t servingCellId = 0;

  // Serving measurements
  double servingRsrpDbm = std::numeric_limits<double>::quiet_NaN (); // from MeasReport
  double servingSinrDb  = std::numeric_limits<double>::quiet_NaN (); // "reported" = sinrDbCalc

  // Best neighbor candidate (MeasReport)
  uint16_t candCellId = 0; // physCellId (PCI namespace; fine as a feature)
  double   candRsrpDbm = std::numeric_limits<double>::quiet_NaN ();

  // PHY SINR debug
  double rawSinrLinear = std::numeric_limits<double>::quiet_NaN ();
  double sinrDbCalc    = std::numeric_limits<double>::quiet_NaN ();
  
  // A3 consecutive-sample persistence (for 1s sampling)
  uint8_t a3CondStreak = 0;
  bool a3QualifiedPrev = false;

  // KPI pulse state (2-sample confirmation)
  uint8_t kpiCondStreak = 0;
  bool    kpiQualifiedPrev = false;

  // UE position
  Vector pos = Vector (0,0,0);
};

static std::ofstream g_csv;
static double g_hysteresisDb = 2.0;
static Time   g_samplePeriod = Seconds (1.0);

static std::unordered_map<uint64_t, PerUeState> g_state;     // IMSI -> state
static std::unordered_map<uint64_t, uint32_t>   g_imsiToIdx; // IMSI -> UE index
static std::unordered_map<uint16_t, Vector>     g_cellPos;   // CellId -> eNB position

static NodeContainer g_ueNodes;
static NetDeviceContainer g_ueDevs;

static uint32_t
ExtractNodeIdFromContext (const std::string &context)
{
  static const std::regex re ("/NodeList/([0-9]+)/");
  std::smatch m;
  if (std::regex_search (context, m, re))
    {
      return static_cast<uint32_t> (std::stoul (m[1].str ()));
    }
  return std::numeric_limits<uint32_t>::max ();
}

// UE PHY trace: SINR (linear -> dB)
static void
UePhyReportCurrentCellRsrpSinr (std::string context,
                                uint16_t /*cellId*/,
                                uint16_t /*rnti*/,
                                double /*rsrpW*/,          // ignored (always 0 in your build)
                                double sinrLinear,
                                uint8_t /*componentCarrierId*/)
{
  const uint32_t nodeId = ExtractNodeIdFromContext (context);
  if (nodeId == std::numeric_limits<uint32_t>::max ())
    {
      return;
    }

  for (uint32_t u = 0; u < g_ueNodes.GetN (); ++u)
    {
      if (g_ueNodes.Get (u)->GetId () == nodeId)
        {
          Ptr<LteUeNetDevice> ue = g_ueDevs.Get (u)->GetObject<LteUeNetDevice> ();
          if (!ue) return;
          const uint64_t imsi = ue->GetImsi ();

          auto &S = g_state[imsi];
          S.rawSinrLinear = sinrLinear;

          if (sinrLinear > 0)
            {
              S.sinrDbCalc = 10.0 * std::log10 (sinrLinear);
              S.servingSinrDb = S.sinrDbCalc; // keep "reported" as the same value
            }
          else
            {
              S.sinrDbCalc = std::numeric_limits<double>::quiet_NaN ();
              S.servingSinrDb = std::numeric_limits<double>::quiet_NaN ();
            }
          return;
        }
    }
}

// eNB receives UE measurement reports: serving + neighbor RSRP
static void
NotifyRecvMeasurementReport (std::string /*context*/,
                             uint64_t imsi,
                             uint16_t cellId,
                             uint16_t /*rnti*/,
                             LteRrcSap::MeasurementReport msg)
{
  auto &S = g_state[imsi];
  S.servingCellId = cellId;

  // Serving RSRP(dBm) = -140 + rsrpResult
  const uint8_t rsrpEnc = msg.measResults.measResultPCell.rsrpResult;
  S.servingRsrpDbm = -140.0 + (double) rsrpEnc;

  // Best neighbor by RSRP
  uint16_t bestId = 0;
  double bestRsrpDbm = -1e9;

  if (msg.measResults.haveMeasResultNeighCells)
    {
      for (const auto &nc : msg.measResults.measResultListEutra)
        {
          const uint16_t neighPci = nc.physCellId;
          const uint8_t neighRsrpEnc = nc.rsrpResult;
          const double neighRsrpDbm = -140.0 + (double) neighRsrpEnc;

          if (neighRsrpDbm > bestRsrpDbm)
            {
              bestRsrpDbm = neighRsrpDbm;
              bestId = neighPci; // keep PCI as candidate ID feature
            }
        }
    }

  if (bestId != 0)
    {
      S.candCellId = bestId;
      S.candRsrpDbm = bestRsrpDbm;
    }
}

static void
Sample ()
{
  const double t = Simulator::Now ().GetSeconds ();
  const double now = t;

  for (const auto &kv : g_imsiToIdx)
    {
      const uint64_t imsi = kv.first;
      const uint32_t uidx = kv.second;
      auto &S = g_state[imsi];

      Ptr<LteUeNetDevice> ue = g_ueDevs.Get (uidx)->GetObject<LteUeNetDevice> ();

      uint16_t currentId = S.servingCellId;
      if (ue && ue->GetRrc ())
        {
          currentId = ue->GetRrc ()->GetCellId ();
          S.servingCellId = currentId;
        }

      Ptr<MobilityModel> mm = g_ueNodes.Get (uidx)->GetObject<MobilityModel> ();
      if (mm) S.pos = mm->GetPosition ();

      // distance UE -> serving eNB
      double distance = std::numeric_limits<double>::quiet_NaN ();
      bool validServing = false;
      if (currentId != 0)
        {
          auto itp = g_cellPos.find (currentId);
          if (itp != g_cellPos.end ())
            {
              distance = ns3::CalculateDistance (S.pos, itp->second);
              validServing = std::isfinite (distance);
            }
        }

      // Rollback to stable serving-side metric for A3/gap: use serving RSRP directly.
      double currentRssiDbm = std::numeric_limits<double>::quiet_NaN ();
      if (std::isfinite (S.servingRsrpDbm))
        {
          currentRssiDbm = S.servingRsrpDbm;
        }

      // gap_db: cand_rsrp - serving_rsrp
      double gapDb = std::numeric_limits<double>::quiet_NaN ();
      if (std::isfinite (S.candRsrpDbm) && std::isfinite (currentRssiDbm))
        {
          gapDb = S.candRsrpDbm - currentRssiDbm;
        }

      int trigger_a3 = 0;
      int trigger_kpi = 0;

      // Invalid serving => reset trigger state and force triggers 0
      if (!validServing)
        {
          S.a3CondStreak = 0;
          S.a3QualifiedPrev = false;

          S.kpiCondStreak = 0;
          S.kpiQualifiedPrev = false;

          S.candCellId = 0;
          S.candRsrpDbm = std::numeric_limits<double>::quiet_NaN ();
        }
      else
        {
          // ---- A3 trigger (RSRP-to-RSRP + hysteresis + TTT pulse) ----
          // ---- A3 trigger using consecutive-sample persistence at 1s sampling ----
          // Condition: cand_rsrp > serving_rsrp + hysteresis
          bool a3Cond =
            (S.candCellId != 0 &&
             std::isfinite (S.candRsrpDbm) &&
             std::isfinite (S.servingRsrpDbm) &&
             (S.candRsrpDbm > S.servingRsrpDbm + g_hysteresisDb));

          if (a3Cond)
            {
              if (S.a3CondStreak < 255) S.a3CondStreak++;
            }
          else
            {
              S.a3CondStreak = 0;
            }

          // N=2 consecutive samples
          bool a3Qualified = (S.a3CondStreak >= 1);

          // Rising-edge pulse
          if (a3Qualified && !S.a3QualifiedPrev)
            {
              trigger_a3 = 1;
            }
          S.a3QualifiedPrev = a3Qualified;

          // ---- KPI trigger (paper-style RSRP/SINR/distance, 2-sample confirmation) ----
          bool kpiCond = false;
          if (std::isfinite (S.servingRsrpDbm) && (S.servingRsrpDbm < KPI_RSRP_THRESH_DBM)) kpiCond = true;
          if (std::isfinite (S.sinrDbCalc)     && (S.sinrDbCalc     < KPI_SINR_THRESH_DB))  kpiCond = true;
          if (std::isfinite (distance)         && (distance         > KPI_DIST_THRESH_M))   kpiCond = true;

          if (kpiCond)
            {
              if (S.kpiCondStreak < 255) S.kpiCondStreak++;
            }
          else
            {
              S.kpiCondStreak = 0;
            }

          bool kpiQualified = (S.kpiCondStreak >= 2);
          if (kpiQualified && !S.kpiQualifiedPrev) trigger_kpi = 1;
          S.kpiQualifiedPrev = kpiQualified;
        }

      // Safety invariants: never emit triggers in invalid serving
      if (trigger_a3 == 1 && (!validServing || currentId == 0 || !std::isfinite (distance)))
        {
          NS_LOG_ERROR ("BUG: trigger_a3=1 during invalid serving (imsi=" << imsi << ")");
          trigger_a3 = 0;
          S.a3CondStreak = 0;
          S.a3QualifiedPrev = false;
        }
      if (trigger_kpi == 1 && (!validServing || currentId == 0 || !std::isfinite (distance)))
        {
          NS_LOG_ERROR ("BUG: trigger_kpi=1 during invalid serving (imsi=" << imsi << ")");
          trigger_kpi = 0;
          S.kpiCondStreak = 0;
          S.kpiQualifiedPrev = false;
        }

      g_csv << std::fixed << std::setprecision (6)
            << t << ","
            << imsi << ","
            << S.candCellId << ","
            << S.candRsrpDbm << ","
            << currentId << ","
            << currentRssiDbm << ","
            << g_hysteresisDb << ","
            << S.servingRsrpDbm << ","
            << S.servingSinrDb << ","
            << S.rawSinrLinear << ","
            << S.sinrDbCalc << ","
            << distance << ","
            << trigger_a3 << ","
            << trigger_kpi << ","
            << gapDb
            << "\n";
    }

  Simulator::Schedule (g_samplePeriod, &Sample);
}

static void
PlaceEnbs (NodeContainer enbNodes, Rectangle area)
{
  const uint32_t rows = 2, cols = 4;
  double dx = (area.xMax - area.xMin) / (cols + 1);
  double dy = (area.yMax - area.yMin) / (rows + 1);

  for (uint32_t r = 0; r < rows; ++r)
    for (uint32_t c = 0; c < cols; ++c)
      {
        uint32_t idx = r * cols + c;
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator> ();
        pos->Add (Vector (area.xMin + (c + 1) * dx, area.yMin + (r + 1) * dy, 30.0));
        MobilityHelper mh;
        mh.SetPositionAllocator (pos);
        mh.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mh.Install (enbNodes.Get (idx));
      }
}

int
main (int argc, char *argv[])
{
  uint16_t numEnbs = 8, numUes = 50;
  double simTime = 500000.0;

  std::string sumoTrace = "mobility.tcl";
  std::string csvOut = "handover_dataset.csv";

  double ueTxDbm = 26.0, enbTxDbm = 46.0;
  g_hysteresisDb = 2.0;
  uint16_t prbs = 50;
  g_samplePeriod = Seconds (1.0);

  CommandLine cmd;
  cmd.AddValue ("numEnbs", "number of eNBs", numEnbs);
  cmd.AddValue ("numUes",  "number of UEs",  numUes);
  cmd.AddValue ("sumoTrace", "NS-2 mobility trace (.tcl) exported from SUMO", sumoTrace);
  cmd.AddValue ("csvOut", "output CSV path", csvOut);
  cmd.AddValue ("simTime", "simulation time (s)", simTime);
  cmd.AddValue ("hysteresisDb", "handover hysteresis (dB)", g_hysteresisDb);
  cmd.Parse (argc, argv);

  NodeContainer enbNodes; enbNodes.Create (numEnbs);
  g_ueNodes.Create (numUes);

  PlaceEnbs (enbNodes, Rectangle (175.0, 1250.0, 230.0, 830.0));

  Ns2MobilityHelper ns2 (sumoTrace);
  ns2.Install ();

  MobilityHelper fallback;
  fallback.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  for (uint32_t i = 0; i < g_ueNodes.GetN (); ++i)
    {
      if (!g_ueNodes.Get (i)->GetObject<MobilityModel> ())
        {
          fallback.Install (g_ueNodes.Get (i));
        }
    }

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxDbm));
  Config::SetDefault ("ns3::LteUePhy::TxPower",  DoubleValue (ueTxDbm));

  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (prbs));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (prbs));

  lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (g_hysteresisDb));
  lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (160)));

  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  g_ueDevs  = lteHelper->InstallUeDevice (g_ueNodes);

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/"
                   "ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                   MakeCallback (&UePhyReportCurrentCellRsrpSinr));

  InternetStackHelper internet;
  internet.Install (g_ueNodes);

  epcHelper->AssignUeIpv4Address (NetDeviceContainer (g_ueDevs));

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < g_ueNodes.GetN (); ++u)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (g_ueNodes.Get (u)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  for (uint32_t i = 0; i < g_ueDevs.GetN (); ++i)
    {
      Ptr<LteUeNetDevice> ue = g_ueDevs.Get (i)->GetObject<LteUeNetDevice> ();
      g_imsiToIdx[ue->GetImsi ()] = i;
    }

  for (uint32_t j = 0; j < enbDevs.GetN (); ++j)
    {
      Ptr<LteEnbNetDevice> enb = enbDevs.Get (j)->GetObject<LteEnbNetDevice> ();
      const uint16_t cid = enb->GetCellId ();
      Ptr<MobilityModel> mm = enbNodes.Get (j)->GetObject<MobilityModel> ();
      g_cellPos[cid] = mm->GetPosition ();
    }

  for (uint32_t i = 0; i < g_ueDevs.GetN (); ++i)
    {
      lteHelper->Attach (g_ueDevs.Get (i));
    }
  lteHelper->AddX2Interface (enbNodes);

  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                   MakeCallback (&NotifyRecvMeasurementReport));

  g_csv.open (csvOut.c_str ());
  g_csv << "time_s,ue_imsi,rssi_id,rssi,current_id,current_rssi,hysteresis,"
           "rsrp,sinr_db_reported,sinr_linear,sinr_db_calc,"
           "distance_ue,trigger_a3,trigger_kpi,gap_db\n";
  g_csv.setf (std::ios::fixed);
  g_csv << std::setprecision (6);

  Simulator::Schedule (g_samplePeriod, &Sample);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  g_csv.close ();
  return 0;
}
