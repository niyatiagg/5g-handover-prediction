/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * lte_sumo_ho_dataset_extractor.cc
 *
 * Outputs (per UE, every 40ms):
 * ueId, time, rssi, current_id, current_rssi, hysteresis, rsrp, sinr, distance_ue, trigger
 *
 * Candidate id/rssi are taken from neighbor measurements inside MeasurementReport.
 * Serving rsrp/sinr from UE PHY trace (ReportCurrentCellRsrpSinr) and/or MeasurementReport PCell.
 * Trigger is latched on EnbRrc/HandoverStart (one sample tick).
 *
 * Build/run (ns-3 waf):
 *   ./waf --run "lte_sumo_ho_dataset_extractor --sumoTrace=mobility.tcl --simTime=500000"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/ipv4-static-routing-helper.h"

#include <fstream>
#include <unordered_map>
#include <iomanip>
#include <cmath>
#include <limits>
#include <regex>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteSumoHoDatasetExtractor");

static inline double DbmToMilliwatt (double dbm) { return std::pow (10.0, dbm / 10.0); }
static inline double MilliwattToDbm (double mw)  { return 10.0 * std::log10 (std::max (mw, 1e-15)); }

struct PerUeState
{
  uint16_t servingCellId = 0;

  // Serving measurements (dBm / dB)
  double servingRsrpDbm = std::numeric_limits<double>::quiet_NaN ();
  double servingSinrDb  = std::numeric_limits<double>::quiet_NaN ();

  // Best neighbor candidate (paper: id, rssi)
  uint16_t candCellId = 0;
  double candRssiDbm  = std::numeric_limits<double>::quiet_NaN ();

  // Trigger latch (1 for the next sample after HO starts)
  bool trigger = false;
  
  double a3CondStartTime = -1.0;  // seconds; -1 means A3 condition not active
  bool a3PulseFired = false;      // ensures pulse behavior (only once per condition episode)


  // Latest UE position
  Vector pos = Vector (0,0,0);
};

static std::ofstream g_csv;
static double g_hysteresisDb = 3.0;
static Time   g_samplePeriod = Seconds (1.0);
static uint64_t g_hoStartCount = 0;

static std::unordered_map<uint64_t, PerUeState> g_state;     // IMSI -> state
static std::unordered_map<uint64_t, uint32_t>   g_imsiToIdx; // IMSI -> UE index
static std::unordered_map<uint16_t, Vector>     g_cellPos;   // CellId -> eNB position

static NodeContainer g_ueNodes;
static NetDeviceContainer g_ueDevs;

static uint32_t
ExtractNodeIdFromContext (const std::string &context)
{
  // context looks like: "/NodeList/7/DeviceList/0/LteUePhy/ReportCurrentCellRsrpSinr"
  static const std::regex re ("/NodeList/([0-9]+)/");
  std::smatch m;
  if (std::regex_search (context, m, re))
    {
      return static_cast<uint32_t> (std::stoul (m[1].str ()));
    }
  return std::numeric_limits<uint32_t>::max ();
}


// ---------- Callbacks ----------

// UE PHY trace: current serving cell RSRP/SINR
// Signature matches LteUePhy::RsrpSinrTracedCallback with context prefix:
// (context, cellId, rnti, rsrp, sinr, componentCarrierId)

static void
UePhyReportCurrentCellRsrpSinr (std::string context,
                                uint16_t /*cellId*/,
                                uint16_t /*rnti*/,
                                double rsrpDbm,
                                double sinrLinear,
                                uint8_t /*componentCarrierId*/)
{
  const uint32_t nodeId = ExtractNodeIdFromContext (context);
  if (nodeId == std::numeric_limits<uint32_t>::max ())
    {
      return;
    }

  // Your UE nodes are NodeContainer g_ueNodes created sequentially, typically nodeIds start after eNBs.
  // So we map nodeId -> UE index by searching g_ueNodes.
  for (uint32_t u = 0; u < g_ueNodes.GetN (); ++u)
    {
      if (g_ueNodes.Get (u)->GetId () == nodeId)
        {
          Ptr<LteUeNetDevice> ue = g_ueDevs.Get (u)->GetObject<LteUeNetDevice> ();
          if (!ue) return;
          const uint64_t imsi = ue->GetImsi ();

          auto &S = g_state[imsi];
          double rsrpW = rsrpDbm; // rename mentally: it's power in W
          double rsrpDbmConv = std::numeric_limits<double>::quiet_NaN();
          if (rsrpW > 0)
          {
            rsrpDbmConv = 10.0 * std::log10(rsrpW * 1000.0); // W -> dBm
          }
          S.servingRsrpDbm = rsrpDbmConv;


          // convert SINR linear -> dB
          if (sinrLinear > 0)
            {
              S.servingSinrDb = 10.0 * std::log10 (sinrLinear);
            }
          
          /*if (Simulator::Now ().GetSeconds () < 2.0)
            {
              std::cout << "t=" << Simulator::Now ().GetSeconds ()
                        << " imsi=" << imsi
                        << " rsrpDbm=" << rsrpDbm
                        << " sinrDb=" << S.servingSinrDb
                        << std::endl;
            }*/
          return;
        }
    }
}


// eNB receives UE measurement reports: contains PCell and neighbor cells
static void
NotifyRecvMeasurementReport (std::string /*context*/,
                             uint64_t imsi,
                             uint16_t cellId,
                             uint16_t /*rnti*/,
                             LteRrcSap::MeasurementReport msg)
{
  auto &S = g_state[imsi];
  S.servingCellId = cellId;

  // Serving cell RSRP (encoded): RSRP(dBm) = -140 + rsrpResult
  const uint8_t rsrpEnc = msg.measResults.measResultPCell.rsrpResult;
  S.servingRsrpDbm = -140.0 + (double) rsrpEnc;

  // Best neighbor candidate by RSRP
  uint16_t bestId = 0;
  double bestRsrpDbm = -1e9;

  if (msg.measResults.haveMeasResultNeighCells)
    {
      for (const auto &nc : msg.measResults.measResultListEutra)
        {
          const uint16_t neighPci = nc.physCellId;

          // ns-3.36 typically stores neighbor rsrp directly as nc.rsrpResult
          const uint8_t neighRsrpEnc = nc.rsrpResult;
          const double neighRsrpDbm = -140.0 + (double) neighRsrpEnc;

          const uint16_t neighCellId = neighPci;

          if (neighRsrpDbm > bestRsrpDbm)
            {
              bestRsrpDbm = neighRsrpDbm;
              bestId = neighCellId;
            }
        }
    }

  if (bestId != 0)
    {
      S.candCellId = bestId;
      S.candRssiDbm = bestRsrpDbm; // using neighbor RSRP as RSSI proxy
    }
}

// HO trigger latch (one-shot)
static void
OnHandoverStart (std::string /*context*/,
                 uint64_t imsi,
                 uint16_t /*cellId*/,
                 uint16_t /*rnti*/,
                 uint16_t /*targetCellId*/)
{
  g_state[imsi].trigger = true;
  g_hoStartCount++;
}


// ---------- Periodic sampling ----------

static void
Sample ()
{
  const double t = Simulator::Now ().GetSeconds ();

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

      // UE position
      Ptr<MobilityModel> mm = g_ueNodes.Get (uidx)->GetObject<MobilityModel> ();
      if (mm) S.pos = mm->GetPosition ();

      // distance_ue: Euclidean distance UE -> serving gNB (paper wording)
      double distance = std::numeric_limits<double>::quiet_NaN ();
      auto itp = g_cellPos.find (currentId);
      if (itp != g_cellPos.end ())
        {
          distance = CalculateDistance (S.pos, itp->second);
        }

      // current_rssi derived from serving rsrp + sinr if available
      double currentRssiDbm = std::numeric_limits<double>::quiet_NaN ();
      if (std::isfinite (S.servingRsrpDbm) && std::isfinite (S.servingSinrDb))
        {
          const double rsrpMw  = DbmToMilliwatt (S.servingRsrpDbm);
          const double sinrLin = std::pow (10.0, S.servingSinrDb / 10.0);
          const double inMw    = (sinrLin > 1e-12) ? (rsrpMw / sinrLin) : 0.0;
          currentRssiDbm       = MilliwattToDbm (rsrpMw + inMw);
        }

      // --- A3 + TTT pulse label (TTT = 160 ms) ---
      const double now = Simulator::Now().GetSeconds();
      const double ttt = 0.160; // 160 ms

      int trigger = 0;

      // A3 condition: candidate better than current by hysteresis
      bool a3Cond =
        (S.candCellId != 0 &&
         std::isfinite(S.candRssiDbm) &&
         std::isfinite(currentRssiDbm) &&
         (S.candRssiDbm > currentRssiDbm + g_hysteresisDb));

      if (a3Cond)
      {
        if (S.a3CondStartTime < 0.0)
        {
          S.a3CondStartTime = now;
          S.a3PulseFired = false;
        }

        // Pulse once when TTT is first satisfied
        if (!S.a3PulseFired && (now - S.a3CondStartTime) >= ttt)
        {
          trigger = 1;
          S.a3PulseFired = true;
        }
      }
      else
      {
        // reset when condition no longer holds
        S.a3CondStartTime = -1.0;
        S.a3PulseFired = false;
      }


      // If no neighbor candidate is known yet, leave candId/candRssi as NaN/0
      g_csv << std::fixed << std::setprecision (6)
            << t << ","
            << imsi << ","
            << S.candCellId << ","
            << S.candRssiDbm << ","
            << currentId << ","
            << currentRssiDbm << ","
            << g_hysteresisDb << ","
            << S.servingRsrpDbm << ","
            << S.servingSinrDb << ","
            << distance << ","
            << trigger
            << "\n";

      
    }

  Simulator::Schedule (g_samplePeriod, &Sample);
}

// ---------- eNB placement helper ----------
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
  uint16_t numEnbs = 8, numUes = 10;
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

  // eNB positions from paper’s rectangle
  PlaceEnbs (enbNodes, Rectangle (175.0, 1250.0, 230.0, 830.0));

  // Save CellId -> position map after devices are installed (we fill after InstallEnbDevice)

  // SUMO .tcl mobility for UEs
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
  lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (40)));

  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  g_ueDevs  = lteHelper->InstallUeDevice (g_ueNodes);
  
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/"
                 "ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                 MakeCallback (&UePhyReportCurrentCellRsrpSinr));
  
    // ---- Internet stack on UEs (required for EPC bearer activation) ----
  InternetStackHelper internet;
  internet.Install (g_ueNodes);

  // Assign IPv4 addresses to UEs
  Ipv4InterfaceContainer ueIpIfaces =
      epcHelper->AssignUeIpv4Address (NetDeviceContainer (g_ueDevs));

  // Set default route for each UE to the EPC's default gateway
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < g_ueNodes.GetN (); ++u)
  {
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting (g_ueNodes.Get (u)->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
  }

  // Build IMSI -> UE index map
  for (uint32_t i = 0; i < g_ueDevs.GetN (); ++i)
    {
      Ptr<LteUeNetDevice> ue = g_ueDevs.Get (i)->GetObject<LteUeNetDevice> ();
      g_imsiToIdx[ue->GetImsi ()] = i;
    }

  // Build CellId -> eNB position map
  for (uint32_t j = 0; j < enbDevs.GetN (); ++j)
    {
      Ptr<LteEnbNetDevice> enb = enbDevs.Get (j)->GetObject<LteEnbNetDevice> ();
      const uint16_t cid = enb->GetCellId ();
      Ptr<MobilityModel> mm = enbNodes.Get (j)->GetObject<MobilityModel> ();
      g_cellPos[cid] = mm->GetPosition ();
    }

  // Attach UEs
  for (uint32_t i = 0; i < g_ueDevs.GetN (); ++i)
    {
      lteHelper->Attach (g_ueDevs.Get (i));
    }
  lteHelper->AddX2Interface (enbNodes);

  // Connect traces
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                   MakeCallback (&NotifyRecvMeasurementReport));

  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&OnHandoverStart));

  // If your ns-3 build exposes this trace, you can also connect it:
  // Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
  //                  MakeCallback (&UePhyRsrpSinr));

  // CSV output
  g_csv.open (csvOut.c_str ());
  g_csv << "time_s,ue_imsi,rssi_id,rssi,current_id,current_rssi,hysteresis,rsrp,sinr,distance_ue,trigger\n";
  g_csv.setf (std::ios::fixed);
  g_csv << std::setprecision (6);
  
  std::cout << "HandoverStart count = " << g_hoStartCount << std::endl;

  Simulator::Schedule (g_samplePeriod, &Sample);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  g_csv.close ();
  return 0;
}

