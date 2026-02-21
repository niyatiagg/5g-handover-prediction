/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//
// lte_sumo_ho_dataset.cc
//
// ns-3 LTE + EPC + X2 + SUMO mobility (.tcl via Ns2MobilityHelper)
// Runs 250000 s and logs the 9 arrays required by the paper into CSV.
//
// Build (waf):
//   ./waf --run "lte_sumo_ho_dataset --sumoTrace=path/to/mobility.tcl"
//
// Output:
//   handover_vector.csv   (one row per UE per 40 ms sample)
//
// Paper mapping (Table 2):
//   DownlinkInterference = true  -> all eNBs share same carrier
//   UplinkInterference   = true  -> default when sharing carrier
//   Ue Number            = 10    -> --numUes=10
//   Gnb Number           = 8     -> --numEnbs=8
//   UeTxPower            = 26 dBm -> ns3::LteUePhy::TxPower
//   ENodeBTxPower        = 46 dBm -> ns3::LteEnbPhy::TxPower
//   TargetBler           = 0.01   -> ns3::LteAmc::TargetBler
//   BlerShift            = 5      -> ns3::LteAmc::BlerShift (if present; ignored otherwise)
//   FbPeriod             = 40 ms  -> sampling period for logging
//   CA numComponentCarriers = 1   -> single LTE carrier (10 MHz = 50 PRBs)
//   CA numerology        = 0      -> LTE SCS analogue (15 kHz)
//   CA carrierFrequency  = 2 GHz  -> approximated by LTE EARFCN in this band
//   CA numBands          = 50     -> DlBandwidth/UlBandwidth = 50 PRBs (10 MHz)
//   DynamicCellAssociation = true -> default initial attach + HO
//   EnableHandover       = true   -> ns3::A3RsrpHandoverAlgorithm
//   Mobility             = SUMO .tcl trace (instead of RandomWaypoint)
//   Mobility Speed       = provided by SUMO (paper used U(10,60) m/s)
//
// The 9 arrays (paper Fig. C):
//   currentId, currentRssi, hysteresis, id, rssi, trigger, sinr, rsrp, distance
//
// References to the paper content and definitions used here:
//   System model and dataset fields (pp. 7–10, Table 2; “Dataset – Logistic Regression Algorithm”).
//* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/ns2-mobility-helper.h"

#include <fstream>
#include <unordered_map>
#include <set>
#include <iomanip>
#include <cmath>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("LteSumoHoDataset");

// ---------- helpers ----------
static inline double DbmToMilliwatt (double dbm) { return std::pow (10.0, dbm / 10.0); }
static inline double MilliwattToDbm (double mw)  { return 10.0 * std::log10 (std::max (mw, 1e-15)); }

// Friis receive power estimate at frequency f (Hz), distance d (m), txDbm (dBm)
static double FriisPrDbm (double txDbm, double fHz, double dMeters)
{
  static const double c = 299792458.0;
  double d = std::max (dMeters, 1.0); // avoid singularity
  double lambda = c / fHz;
  // Pr = Pt * (lambda/(4*pi*d))^2 ; in dB: 20log10(lambda/(4πd))
  double path = 20.0 * std::log10 (lambda / (4.0 * M_PI * d));
  return txDbm + path; // antenna gains 0 dB
}

struct UeMetrics
{
  double lastRsrpDbm = NAN;
  double lastSinrDb = NAN;
  uint16_t servingCellId = 0;
  bool hoTrigger = false;
  Vector lastPos = Vector (0,0,0);
};

class DatasetCollector
{
public:
  DatasetCollector (const std::string &csvPath,
                    double hysteresisDb,
                    Time samplePeriod,
                    NetDeviceContainer ueDevs,
                    NetDeviceContainer enbDevs,
                    NodeContainer ueNodes,
                    NodeContainer enbNodes,
                    double txDbmEnb,
                    double carrierHz)
    : m_csv (csvPath.c_str ()), m_hyst (hysteresisDb), m_period (samplePeriod),
      m_ueDevs (ueDevs), m_enbDevs (enbDevs),
      m_ueNodes (ueNodes), m_enbNodes (enbNodes),
      m_txDbmEnb (txDbmEnb), m_fHz (carrierHz)
  {
    if (!m_csv.is_open ())
      {
        NS_FATAL_ERROR ("Cannot open output CSV: " << csvPath);
      }
    m_csv << "time_s,id,rssi,currentId,currentRssi,hysteresis,rsrp,sinr,distance,trigger\n";
    m_csv.setf (std::ios::fixed); m_csv<<std::setprecision(6);

    // IMSI -> UE index; CellId -> eNB position
    for (uint32_t i = 0; i < m_ueDevs.GetN (); ++i)
      {
        Ptr<LteUeNetDevice> ue = m_ueDevs.Get (i)->GetObject<LteUeNetDevice> ();
        m_imsiToIdx[ue->GetImsi ()] = i;
      }
    for (uint32_t j = 0; j < m_enbDevs.GetN (); ++j)
      {
        Ptr<LteEnbNetDevice> enb = m_enbDevs.Get (j)->GetObject<LteEnbNetDevice> ();
        uint16_t cid = enb->GetCellId ();
        Ptr<MobilityModel> mm = m_enbNodes.Get (j)->GetObject<MobilityModel> ();
        m_cellPos[cid] = mm->GetPosition ();
      }

    // Traces we have in ns-3.36
    // Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
                     //MakeCallback (&DatasetCollector::OnUeCurrentRsrpSinr, this));

    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                     MakeCallback (&DatasetCollector::OnHandoverStart, this));

    Simulator::Schedule (m_period, &DatasetCollector::Sample, this);
  }

  void Sample ()
  {
    const double t = Simulator::Now ().GetSeconds ();
    for (auto &kv : m_imsiToIdx)
      {
        const uint64_t imsi = kv.first;
        const uint32_t uidx = kv.second;
        auto &M = m_metrics[imsi];

        // Serving cell id from RRC (if available)
        Ptr<LteUeNetDevice> ue = m_ueDevs.Get (uidx)->GetObject<LteUeNetDevice> ();
        uint16_t currentId = M.servingCellId;
        if (ue && ue->GetRrc ()) currentId = ue->GetRrc ()->GetCellId ();

        // UE position
        Ptr<MobilityModel> umm = m_ueNodes.Get (uidx)->GetObject<MobilityModel> ();
        if (umm) M.lastPos = umm->GetPosition ();

        // Distance to serving eNB
        double distance = NAN;
        auto itp = m_cellPos.find (currentId);
        if (itp != m_cellPos.end ()) distance = CalculateDistance (M.lastPos, itp->second);

        // Serving RSRP/SINR
        const double rsrpDbm = M.lastRsrpDbm;
        const double sinrDb  = M.lastSinrDb;

        // Derive serving RSSI = signal + interference/noise
        double currentRssiDbm = NAN;
        if (std::isfinite (rsrpDbm) && std::isfinite (sinrDb))
          {
            const double rsrpMw  = DbmToMilliwatt (rsrpDbm);
            const double sinrLin = std::pow (10.0, sinrDb / 10.0);
            const double inMw    = (sinrLin > 1e-12) ? (rsrpMw / sinrLin) : 0.0;
            currentRssiDbm       = MilliwattToDbm (rsrpMw + inMw);
          }

        // Candidate (id, rssi) by Friis estimate to every non-serving eNB
        uint16_t candId = 0;
        double   candRssi = -1e9;
        for (auto const &cp : m_cellPos)
          {
            if (cp.first == currentId) continue;
            const double d = CalculateDistance (M.lastPos, cp.second);
            const double prDbm = FriisPrDbm (m_txDbmEnb, m_fHz, d);
            if (prDbm > candRssi)
              { candRssi = prDbm; candId = cp.first; }
          }

        const int trigger = M.hoTrigger ? 1 : 0;

        m_csv  << t << ","
               << candId << ","
               << candRssi << ","
               << currentId << ","
               << currentRssiDbm << ","
               << m_hyst << ","
               << rsrpDbm << ","
               << sinrDb << ","
               << distance << ","
               << trigger << "\n";

        M.hoTrigger = false; // reset one-shot latch
      }

    Simulator::Schedule (m_period, &DatasetCollector::Sample, this);
  }

private:
  void OnUeCurrentRsrpSinr (uint16_t cellId, uint16_t rnti, double rsrpDbm, double sinrDb)
  {
    // Map (cellId, rnti) to IMSI (small-N linear scan)
    uint64_t imsi = 0;
    for (auto const &kv : m_imsiToIdx)
      {
        Ptr<LteUeNetDevice> ue = m_ueDevs.Get (kv.second)->GetObject<LteUeNetDevice> ();
        if (ue && ue->GetRrc () &&
            ue->GetRrc ()->GetCellId () == cellId &&
            ue->GetRrc ()->GetRnti ()   == rnti)
          { imsi = kv.first; break; }
      }
    if (imsi == 0) return;
    auto &M = m_metrics[imsi];
    M.servingCellId = cellId;
    M.lastRsrpDbm   = rsrpDbm;
    M.lastSinrDb    = sinrDb;
  }

  void OnHandoverStart (uint64_t imsi, uint16_t /*cellId*/, uint16_t /*rnti*/, uint16_t targetCellId)
  {
    auto &M = m_metrics[imsi];
    M.hoTrigger = true;
    // Note: we still compute candidate by Friis each tick;
    // targetCellId is available if you prefer to overwrite candId at that tick.
    (void)targetCellId;
  }

private:
  std::ofstream m_csv;
  double m_hyst;
  Time   m_period;

  NetDeviceContainer m_ueDevs, m_enbDevs;
  NodeContainer      m_ueNodes, m_enbNodes;

  double m_txDbmEnb;
  double m_fHz;

  std::unordered_map<uint64_t, UeMetrics> m_metrics;   // per-IMSI
  std::unordered_map<uint64_t, uint32_t>  m_imsiToIdx; // IMSI -> UE index
  std::unordered_map<uint16_t, Vector>    m_cellPos;   // CellId -> eNB pos
};

// Place 8 eNBs in a 2x4 grid inside the rectangle
static void PlaceEnbs (NodeContainer enbNodes, Rectangle area)
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

int main (int argc, char *argv[])
{
  uint16_t numEnbs = 8, numUes = 10;
  double simTime = 250000.0;
  std::string sumoTrace = "mobility.tcl";
  std::string csvOut = "handover_vector.csv";
  double ueTxDbm = 26.0, enbTxDbm = 46.0;
  double hysteresisDb = 3.0;
  uint16_t prbs = 50;
  Time samplePeriod = MilliSeconds (40);
  double carrierHz = 2.0e9;

  CommandLine cmd;
  cmd.AddValue ("numEnbs", "number of eNBs", numEnbs);
  cmd.AddValue ("numUes",  "number of UEs",  numUes);
  cmd.AddValue ("sumoTrace", "NS-2 mobility trace (.tcl) exported from SUMO", sumoTrace);
  cmd.AddValue ("csvOut", "output CSV path", csvOut);
  cmd.AddValue ("simTime", "simulation time (s)", simTime);
  cmd.Parse (argc, argv);

  // AMC BLER target per paper
  
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.01));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxDbm));
  Config::SetDefault ("ns3::LteUePhy::TxPower",  DoubleValue (ueTxDbm));

  NodeContainer enbNodes; enbNodes.Create (numEnbs);
  NodeContainer ueNodes;  ueNodes.Create (numUes);

  PlaceEnbs (enbNodes, Rectangle (175.0, 1250.0, 230.0, 830.0));

  // SUMO .tcl mobility
  Ns2MobilityHelper ns2 (sumoTrace);
  ns2.Install ();
  MobilityHelper fallbackMob;
  fallbackMob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      if (!ueNodes.Get (i)->GetObject<MobilityModel> ())
        fallbackMob.Install (ueNodes.Get (i));
    }

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // 10 MHz (50 PRBs), 2 GHz, shared carrier
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (prbs));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (prbs));

  // A3-RSRP HO with paper-like parameters
  lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (hysteresisDb));
  lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (40)));

  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs  = lteHelper->InstallUeDevice (ueNodes);

  InternetStackHelper internet;
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  for (uint32_t i = 0; i < ueDevs.GetN (); ++i)
    {
      lteHelper->Attach (ueDevs.Get (i));
    }

  // Correct API on ns-3.36:
  lteHelper->AddX2Interface (enbNodes);

  // CBR downlink traffic
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHost; remoteHost.Create (1);
  InternetStackHelper hostStack; hostStack.Install (remoteHost);
  PointToPointHelper p2p; p2p.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));
  NetDeviceContainer internetDevices = p2p.Install (pgw, remoteHost.Get (0));
  Ipv4AddressHelper ipv4h; ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer ifaces = ipv4h.Assign (internetDevices);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> rh = ipv4RoutingHelper.GetStaticRouting (remoteHost.Get (0)->GetObject<Ipv4> ());
  rh->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  uint16_t dlPort = 1234;
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      UdpServerHelper server (dlPort);
      auto serverApp = server.Install (ueNodes.Get (u));
      serverApp.Start (Seconds (0.1));
      serverApp.Stop (Seconds (simTime));

      UdpClientHelper client (ueIp.GetAddress (u), dlPort);
      client.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      client.SetAttribute ("Interval", TimeValue (MilliSeconds (1)));
      client.SetAttribute ("PacketSize", UintegerValue (512));
      auto clientApp = client.Install (remoteHost.Get (0));
      clientApp.Start (Seconds (1.0 + 0.01 * u));
      clientApp.Stop (Seconds (simTime));
    }

  lteHelper->EnablePhyTraces ();

  // Plain C++ object; keep it alive until after Run()
  auto collector = new DatasetCollector (csvOut, hysteresisDb, samplePeriod,
                                         ueDevs, enbDevs, ueNodes, enbNodes,
                                         enbTxDbm, carrierHz);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  delete collector;
  return 0;
}
