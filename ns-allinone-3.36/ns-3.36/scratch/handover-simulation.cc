/*
 * Handover Simulation for 5G Networks
 * Based on: "Reducing Unnecessary Handovers Using Logistic Regression in 5G Networks"
 * Authors: ALISON M. FERNANDES, HERMES I. DEL MONEGO, BRUNO S. CHANG, AND ANELISE MUNARETTO
 * 
 * This simulation uses ns3 and SUMO for generating a dataset for handover analysis
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/lte-module.h"
// Ns2MobilityHelper is included in mobility-module.h
// #include "ns3/ns2-mobility-helper.h"  // Uncomment if separate header needed
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/log.h"
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <limits>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("HandoverSimulation");

// Global variables for data collection
std::ofstream handoverFile;
std::ofstream measurementFile;
std::map<uint64_t, uint16_t> ueCurrentCell; // IMSI -> Cell ID
std::map<uint64_t, Vector> ueLastPosition; // IMSI -> Last position
std::map<uint64_t, double> ueLastSpeed; // IMSI -> Last speed
std::map<uint64_t, double> ueThroughputDl; // IMSI -> Downlink throughput (Mbps)
std::map<uint64_t, double> ueThroughputUl; // IMSI -> Uplink throughput (Mbps)
std::map<uint64_t, double> ueLastRsrp; // IMSI -> Last RSRP value (dBm)
std::map<uint64_t, double> ueLastRsrq; // IMSI -> Last RSRQ value (dB)
std::map<uint64_t, double> ueLastSinr; // IMSI -> Last SINR value (dB)
std::map<uint16_t, double> cellRsrp; // CellId -> Latest RSRP (for PHY trace)
std::map<uint16_t, double> cellSinr; // CellId -> Latest SINR (for PHY trace)

// CSV Data Logger Class
class HandoverDataLogger : public Object
{
public:
  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("HandoverDataLogger")
      .SetParent<Object> ()
      .AddConstructor<HandoverDataLogger> ();
    return tid;
  }

  HandoverDataLogger ()
  {
    // Default constructor for Object factory
  }

  void Initialize (std::string filename)
  {
    m_file.open (filename);
    // Write CSV header
    m_file << "Time,UE_ID,Old_gNB_ID,New_gNB_ID,RSRP_Old,RSRP_New,"
           << "RSRQ_Old,RSRQ_New,SINR_Old,SINR_New,Throughput_DL,Throughput_UL,"
           << "X_Position,Y_Position,Speed,Handover_Type\n";
  }

  virtual ~HandoverDataLogger ()
  {
    if (m_file.is_open ())
      {
        m_file.close ();
      }
  }

  void LogHandover (double time, uint64_t ueId, uint16_t oldCellId, uint16_t newCellId,
                    double rsrpOld, double rsrpNew, double rsrqOld, double rsrqNew,
                    double sinrOld, double sinrNew,
                    double throughputDl, double throughputUl,
                    double x, double y, double speed, std::string hoType)
  {
    m_file << std::fixed << std::setprecision (6)
           << time << "," << ueId << "," << oldCellId << "," << newCellId << ","
           << rsrpOld << "," << rsrpNew << "," << rsrqOld << "," << rsrqNew << ","
           << sinrOld << "," << sinrNew << ","
           << throughputDl << "," << throughputUl << ","
           << x << "," << y << "," << speed << "," << hoType << "\n";
    m_file.flush ();
  }

  void LogMeasurement (double time, uint64_t ueId, uint16_t cellId,
                       double rsrp, double rsrq, double sinr, double x, double y, double speed,
                       double throughputDl = 0.0, double throughputUl = 0.0)
  {
    // For MEASUREMENT rows: fill both Old and New columns with serving cell
    // and place RSRP/RSRQ/SINR in the "_New" columns (not "_Old")
    m_file << std::fixed << std::setprecision (6)
           << time << "," << ueId << "," << cellId << "," << cellId << ","
           << "," << rsrp << "," << "," << rsrq << "," << "," << sinr << ","
           << throughputDl << "," << throughputUl << ","
           << x << "," << y << "," << speed << ",MEASUREMENT\n";
    m_file.flush ();
  }

private:
  std::ofstream m_file;
};

// Global logger instance
Ptr<HandoverDataLogger> g_logger;

// Handover callback functions
void NotifyHandoverStartEnb (std::string context,
                              uint64_t imsi,
                              uint16_t cellid,
                              uint16_t rnti,
                              uint16_t targetCellId)
{
  NS_LOG_INFO ("Handover start: IMSI " << imsi
                 << " from cell " << cellid << " to cell " << targetCellId);
  
  // Track the current cell before handover (so we can use it in HandoverEndOk)
  if (ueCurrentCell.find (imsi) == ueCurrentCell.end ())
    {
      // First time we see this UE, set current cell
      ueCurrentCell[imsi] = cellid;
    }
  // Note: We don't update ueCurrentCell here because the handover hasn't completed yet
  // It will be updated in NotifyHandoverEndOkEnb
}

void NotifyHandoverEndOkEnb (std::string context,
                              uint64_t imsi,
                              uint16_t cellid,
                              uint16_t rnti)
{
  double time = Simulator::Now ().GetSeconds ();
  
  // Note: The HandoverEndOk trace only provides: imsi, cellid (current cell), rnti
  // The target cell ID is not provided by this trace. We can infer it from the
  // current cell association or use the cellid as both old and new.
  uint16_t targetCellId = cellid; // Current cell after handover
  uint16_t oldCellId = cellid;    // We don't have the old cell from this trace
  
  // Try to get the old cell from our tracking map
  if (ueCurrentCell.find (imsi) != ueCurrentCell.end ())
    {
      oldCellId = ueCurrentCell[imsi];
      targetCellId = cellid; // New cell is the current cellid
    }
  
  NS_LOG_INFO ("Handover completed: IMSI " << imsi
                 << " to cell " << cellid);
  
  // Log handover event (with default values - these would need to be extracted from traces)
  if (g_logger != 0)
    {
      double x = 0, y = 0, speed = 0;
      if (ueLastPosition.find (imsi) != ueLastPosition.end ())
        {
          x = ueLastPosition[imsi].x;
          y = ueLastPosition[imsi].y;
        }
      if (ueLastSpeed.find (imsi) != ueLastSpeed.end ())
        {
          speed = ueLastSpeed[imsi];
        }
      
      // Get latest RSRP/RSRQ/SINR values
      double rsrpOld = -100.0;
      double rsrpNew = -100.0;
      double rsrqOld = -20.0;
      double rsrqNew = -20.0;
      double sinrOld = 0.0;
      double sinrNew = 0.0;
      
      if (ueLastRsrp.find (imsi) != ueLastRsrp.end ())
        {
          rsrpNew = ueLastRsrp[imsi]; // Use current as new
          rsrpOld = rsrpNew; // For now, use same (would need historical tracking)
        }
      if (ueLastRsrq.find (imsi) != ueLastRsrq.end ())
        {
          rsrqNew = ueLastRsrq[imsi];
          rsrqOld = rsrqNew;
        }
      if (ueLastSinr.find (imsi) != ueLastSinr.end ())
        {
          sinrNew = ueLastSinr[imsi];
          sinrOld = sinrNew;
        }
      
      // Get throughput
      double throughputDl = 0.0;
      double throughputUl = 0.0;
      if (ueThroughputDl.find (imsi) != ueThroughputDl.end ())
        {
          throughputDl = ueThroughputDl[imsi];
        }
      if (ueThroughputUl.find (imsi) != ueThroughputUl.end ())
        {
          throughputUl = ueThroughputUl[imsi];
        }
      
      g_logger->LogHandover (time, imsi, oldCellId, targetCellId,
                             rsrpOld, rsrpNew, rsrqOld, rsrqNew,
                             sinrOld, sinrNew,
                             throughputDl, throughputUl,
                             x, y, speed, "HANDOVER");
      
      // Update current cell
      ueCurrentCell[imsi] = targetCellId;
    }
}

void NotifyConnectionEstablishedUe (std::string context,
                                    uint64_t imsi,
                                    uint16_t cellid,
                                    uint16_t rnti)
{
  NS_LOG_INFO ("Connection established: IMSI " << imsi
                 << " to cell " << cellid);
  ueCurrentCell[imsi] = cellid;
}

// PHY trace callback for RSRP and SINR (more accurate than RRC measurement reports)
// Signature matches LteUePhy::RsrpSinrTracedCallback: (cellId, rnti, rsrp, sinr, componentCarrierId)
// When using Config::Connect (not ConnectWithoutContext), NS3 adds context as first parameter
void UePhyRsrpSinr (std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId)
{
  // Store latest RSRP and SINR by cellId
  // In the periodic logger, we'll match these to IMSI based on current cell association
  cellRsrp[cellId] = rsrp;
  cellSinr[cellId] = sinr;
  
  // Also try to update IMSI-specific maps by parsing context
  // Context format: /NodeList/X/DeviceList/Y/LteUePhy/ReportCurrentCellRsrpSinr
  size_t nodeListPos = context.find ("/NodeList/");
  size_t deviceListPos = context.find ("/DeviceList/");
  
  if (nodeListPos != std::string::npos && deviceListPos != std::string::npos)
    {
      // Extract node index for potential IMSI lookup
      // For now, we store by cellId and match in periodic logger
      (void)nodeListPos; // Suppress unused warning
      (void)deviceListPos; // Suppress unused warning
    }
}

// Measurement report callback (from eNB side - receives UE measurement reports)
// Note: This uses the ReceiveReportTracedCallback signature
// Used primarily for RSRQ which may not be available in PHY traces
void NotifyRecvMeasurementReport (std::string context,
                                   uint64_t imsi,
                                   uint16_t cellId,
                                   uint16_t rnti,
                                   LteRrcSap::MeasurementReport msg)
{
  double time = Simulator::Now ().GetSeconds ();
  
  // Extract RSRP and RSRQ from measurement report
  double rsrp = -1000.0; // Invalid default
  double rsrq = -1000.0; // Invalid default
  
  // The measurement report contains measResults with measResultPCell (Primary Cell)
  // measResultPCell contains RSRP and RSRQ for the serving cell
  // RSRP and RSRQ are encoded as uint8_t:
  // RSRP = -140 + rsrpResult (in dBm)
  // RSRQ = -19.5 + rsrqResult/2 (in dB)
  
  // Extract RSRP (encoded value, convert to dBm)
  // RSRP encoding: -140 + value (dBm)
  uint8_t rsrpEncoded = msg.measResults.measResultPCell.rsrpResult;
  rsrp = -140.0 + (double)rsrpEncoded;
  ueLastRsrp[imsi] = rsrp;
  
  // Extract RSRQ (encoded value, convert to dB)
  // RSRQ encoding: -19.5 + value/2 (dB)
  uint8_t rsrqEncoded = msg.measResults.measResultPCell.rsrqResult;
  rsrq = -19.5 + (double)rsrqEncoded / 2.0;
  ueLastRsrq[imsi] = rsrq;
  
  // Get SINR from PHY trace (stored by cellId, then matched to IMSI)
  double sinr = 0.0;
  if (ueLastSinr.find (imsi) != ueLastSinr.end ())
    {
      sinr = ueLastSinr[imsi];
    }
  else if (cellId > 0 && cellSinr.find (cellId) != cellSinr.end ())
    {
      // Use cellId-based SINR from PHY trace
      sinr = cellSinr[cellId];
      ueLastSinr[imsi] = sinr; // Cache for this IMSI
    }
  
  // Log measurement with real values
  if (g_logger != 0)
    {
      double x = 0, y = 0, speed = 0;
      if (ueLastPosition.find (imsi) != ueLastPosition.end ())
        {
          x = ueLastPosition[imsi].x;
          y = ueLastPosition[imsi].y;
        }
      if (ueLastSpeed.find (imsi) != ueLastSpeed.end ())
        {
          speed = ueLastSpeed[imsi];
        }
      
      double throughputDl = 0.0;
      double throughputUl = 0.0;
      if (ueThroughputDl.find (imsi) != ueThroughputDl.end ())
        {
          throughputDl = ueThroughputDl[imsi];
        }
      if (ueThroughputUl.find (imsi) != ueThroughputUl.end ())
        {
          throughputUl = ueThroughputUl[imsi];
        }
      
      g_logger->LogMeasurement (time, imsi, cellId, rsrp, rsrq, sinr, x, y, speed, throughputDl, throughputUl);
    }
}

// RSRP measurement callback (alternative - if UE side trace exists)
// Note: This may not be available in all NS3 versions
void NotifyReportUeMeasurements (std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  double rsrp,
                                  double rsrq,
                                  bool servingCell,
                                  uint8_t componentCarrierId)
{
  if (!servingCell)
    return; // Only log serving cell measurements
  
  // Store latest measurement values
  ueLastRsrp[imsi] = rsrp;
  ueLastRsrq[imsi] = rsrq;
  
  double time = Simulator::Now ().GetSeconds ();
  
  if (g_logger != 0)
    {
      double x = 0, y = 0, speed = 0;
      if (ueLastPosition.find (imsi) != ueLastPosition.end ())
        {
          x = ueLastPosition[imsi].x;
          y = ueLastPosition[imsi].y;
        }
      if (ueLastSpeed.find (imsi) != ueLastSpeed.end ())
        {
          speed = ueLastSpeed[imsi];
        }
      
      double throughputDl = 0.0;
      double throughputUl = 0.0;
      if (ueThroughputDl.find (imsi) != ueThroughputDl.end ())
        {
          throughputDl = ueThroughputDl[imsi];
        }
      if (ueThroughputUl.find (imsi) != ueThroughputUl.end ())
        {
          throughputUl = ueThroughputUl[imsi];
        }
      
      // Get SINR from PHY trace (stored by cellId, then matched to IMSI)
      double sinr = 0.0;
      if (ueLastSinr.find (imsi) != ueLastSinr.end ())
        {
          sinr = ueLastSinr[imsi];
        }
      else if (cellId > 0 && cellSinr.find (cellId) != cellSinr.end ())
        {
          // Use cellId-based SINR from PHY trace
          sinr = cellSinr[cellId];
          ueLastSinr[imsi] = sinr; // Cache for this IMSI
        }
      
      g_logger->LogMeasurement (time, imsi, cellId, rsrp, rsrq, sinr, x, y, speed, throughputDl, throughputUl);
    }
}

// Periodic throughput sampling function
// Updates ueThroughputDl and ueThroughputUl maps during simulation
void SampleThroughput (Ptr<FlowMonitor> monitor,
                       Ptr<Ipv4FlowClassifier> classifier,
                       NetDeviceContainer ueDevs,
                       Ipv4InterfaceContainer ueIpIfaces,
                       uint16_t numUes,
                       double period)
{
  monitor->CheckForLostPackets ();
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  
  // Build IP to IMSI mapping (do this once and reuse, but for simplicity we rebuild)
  std::map<Ipv4Address, uint64_t> ipToImsi;
  for (uint16_t i = 0; i < numUes; ++i)
    {
      if (i < ueIpIfaces.GetN ())
        {
          Ipv4Address ueIp = ueIpIfaces.GetAddress (i);
          if (i < ueDevs.GetN ())
            {
              Ptr<NetDevice> ueDev = ueDevs.Get (i);
              Ptr<LteUeNetDevice> lteUeDev = ueDev->GetObject<LteUeNetDevice> ();
              if (lteUeDev)
                {
                  uint64_t imsi = lteUeDev->GetImsi ();
                  ipToImsi[ueIp] = imsi;
                }
            }
        }
    }
  
  // Calculate throughput per UE from flow statistics
  std::map<uint64_t, double> totalThroughputDl;
  std::map<uint64_t, double> totalThroughputUl;
  
  double currentTime = Simulator::Now ().GetSeconds ();
  
  for (auto it = stats.begin (); it != stats.end (); ++it)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (it->first);
      
      if (currentTime > 0)
        {
          double flowThroughputDl = it->second.rxBytes * 8.0 / (currentTime * 1000000.0); // Mbps
          double flowThroughputUl = it->second.txBytes * 8.0 / (currentTime * 1000000.0); // Mbps
          
          // Downlink: packets to UE IP (from remote host)
          if (ipToImsi.find (t.destinationAddress) != ipToImsi.end ())
            {
              uint64_t imsi = ipToImsi[t.destinationAddress];
              totalThroughputDl[imsi] += flowThroughputDl;
            }
          
          // Uplink: packets from UE IP (to remote host)
          if (ipToImsi.find (t.sourceAddress) != ipToImsi.end ())
            {
              uint64_t imsi = ipToImsi[t.sourceAddress];
              totalThroughputUl[imsi] += flowThroughputUl;
            }
        }
    }
  
  // Update global throughput maps
  for (auto it = totalThroughputDl.begin (); it != totalThroughputDl.end (); ++it)
    {
      ueThroughputDl[it->first] = it->second;
    }
  for (auto it = totalThroughputUl.begin (); it != totalThroughputUl.end (); ++it)
    {
      ueThroughputUl[it->first] = it->second;
    }
  
  // Schedule next sample
  Simulator::Schedule (Seconds (period), &SampleThroughput, monitor, classifier, ueDevs, ueIpIfaces, numUes, period);
}

// Periodic logging function that runs exactly on whole seconds
void LogEverySecond (NodeContainer ueNodes, NetDeviceContainer ueDevs)
{
  double time = Simulator::Now ().GetSeconds ();
  
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<Node> node = ueNodes.Get (i);
      Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
      if (mobility)
        {
          Vector pos = mobility->GetPosition ();
          Vector vel = mobility->GetVelocity ();
          double speed = std::sqrt (vel.x * vel.x + vel.y * vel.y);
          
          // Get actual IMSI from UE device
          uint64_t imsi = 0;
          if (i < ueDevs.GetN ())
            {
              Ptr<NetDevice> ueDev = ueDevs.Get (i);
              Ptr<LteUeNetDevice> lteUeDev = ueDev->GetObject<LteUeNetDevice> ();
              if (lteUeDev)
                {
                  imsi = lteUeDev->GetImsi ();
                }
              else
                {
                  imsi = i + 1; // Fallback if device is not LTE UE device
                }
            }
          else
            {
              imsi = i + 1; // Fallback if index is out of range
            }
          
          ueLastPosition[imsi] = pos;
          ueLastSpeed[imsi] = speed;
          
          // Log periodic measurement
          if (g_logger != 0)
            {
              uint16_t cellId = 0;
              if (ueCurrentCell.find (imsi) != ueCurrentCell.end ())
                {
                  cellId = ueCurrentCell[imsi];
                }
              
              // Log periodic measurement with latest RSRP/RSRQ/SINR values
              // Use stored values if available, otherwise use defaults
              double rsrp = -100.0;
              double rsrq = -20.0;
              double sinr = 0.0;
              
              // First try IMSI-specific values
              if (ueLastRsrp.find (imsi) != ueLastRsrp.end ())
                {
                  rsrp = ueLastRsrp[imsi];
                }
              else if (cellId > 0 && cellRsrp.find (cellId) != cellRsrp.end ())
                {
                  // Fallback: use cellId-based value from PHY trace
                  rsrp = cellRsrp[cellId];
                  ueLastRsrp[imsi] = rsrp; // Cache for this IMSI
                }
              
              if (ueLastRsrq.find (imsi) != ueLastRsrq.end ())
                {
                  rsrq = ueLastRsrq[imsi];
                }
              
              if (ueLastSinr.find (imsi) != ueLastSinr.end ())
                {
                  sinr = ueLastSinr[imsi];
                }
              else if (cellId > 0 && cellSinr.find (cellId) != cellSinr.end ())
                {
                  // Fallback: use cellId-based value from PHY trace
                  sinr = cellSinr[cellId];
                  ueLastSinr[imsi] = sinr; // Cache for this IMSI
                }
              
              // Get throughput if available
              double throughputDl = 0.0;
              double throughputUl = 0.0;
              if (ueThroughputDl.find (imsi) != ueThroughputDl.end ())
                {
                  throughputDl = ueThroughputDl[imsi];
                }
              if (ueThroughputUl.find (imsi) != ueThroughputUl.end ())
                {
                  throughputUl = ueThroughputUl[imsi];
                }
              
              // Log measurement with throughput
              g_logger->LogMeasurement (time, imsi, cellId, rsrp, rsrq, sinr, pos.x, pos.y, speed, throughputDl, throughputUl);
            }
        }
    }
  
  // Schedule next update exactly 1 second later
  Simulator::Schedule (Seconds (1.0), &LogEverySecond, ueNodes, ueDevs);
}

int
main (int argc, char *argv[])
{
  // Simulation parameters from the paper
  uint16_t numUes = 10;
  uint16_t numGnbs = 8;
  double simTime = 100.0; // seconds
  // Default area - can be overridden via CLI to match SUMO trace coordinates
  // SUMO trace typically spans: X:[-1.6, 3226.6], Y:[-1.6, 1201.6]
  double areaXMin = -5.0;  // Changed to match SUMO trace bounds
  double areaXMax = 3230.0;
  double areaYMin = -5.0;
  double areaYMax = 1210.0;
  
  // Radio parameters
  double ueTxPower = 26.0; // dBm
  double gnbTxPower = 46.0; // dBm
  double carrierFreq = 2.0e9; // 2 GHz (CA carrierFrequency)
  
  // Carrier Aggregation (CA) parameters
  uint16_t numComponentCarriers = 1; // CA numComponentCarriers
  uint16_t numerology = 0; // CA numerology
  uint16_t numBands = 50; // CA numBands
  
  // BLER parameters
  double targetBler = 0.01; // TargetBler
  uint16_t blerShift = 5; // BlerShift
  uint16_t fbPeriod = 40; // FbPeriod (Feedback Period)
  
  // Cell ID parameters
  uint16_t macCellId = 0; // MacCellId
  uint16_t masterId = 0; // MasterId
  uint16_t nrMacCellId = 1; // NrMacCellId
  uint16_t nrMasterId = 1; // NrMasterId
  
  // Handover parameters
  bool enableHandover = true;
  bool dynamicCellAssociation = true; // DynamicCellAssociation
  double handoverHysteresis = 3.0; // dB (A3 handover hysteresis)
  uint32_t timeToTriggerMs = 256; // ms (time-to-trigger for handover)
  
  // Interference parameters
  bool downlinkInterference = true; // Downlink Interference
  bool uplinkInterference = true; // Uplink Interference
  
  // Traffic parameters
  double dataRate = 1000000; // 1 Mbps CBR
  uint16_t packetSize = 1024; // bytes
  
  // SUMO integration parameters
  bool useSumo = false; // Set to true if SUMO is configured
  std::string sumoConfigFile = "urban-scenario.sumocfg";
  std::string sumoBinary = "/usr/bin/sumo";
  std::string sumoTraceFile = ""; // Path to SUMO TCL trace file (e.g., "ns3-mobility.tcl")
  
  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numUes", "Number of UEs", numUes);
  cmd.AddValue ("numGnbs", "Number of gNBs", numGnbs);
  cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue ("useSumo", "Use SUMO for mobility", useSumo);
  cmd.AddValue ("sumoConfig", "SUMO configuration file", sumoConfigFile);
  cmd.AddValue ("sumoTrace", "SUMO TCL trace file (for ns2 mobility)", sumoTraceFile);
  cmd.AddValue ("ueTxPower", "UE transmit power (dBm)", ueTxPower);
  cmd.AddValue ("gnbTxPower", "gNB transmit power (dBm)", gnbTxPower);
  cmd.AddValue ("targetBler", "Target BLER", targetBler);
  cmd.AddValue ("blerShift", "BLER shift", blerShift);
  cmd.AddValue ("fbPeriod", "Feedback period", fbPeriod);
  cmd.AddValue ("numComponentCarriers", "Number of component carriers", numComponentCarriers);
  cmd.AddValue ("numerology", "CA numerology", numerology);
  cmd.AddValue ("numBands", "Number of bands", numBands);
  cmd.AddValue ("handoverHysteresis", "Handover hysteresis in dB (default: 3.0)", handoverHysteresis);
  cmd.AddValue ("timeToTrigger", "Handover time-to-trigger in ms (default: 256)", timeToTriggerMs);
  cmd.AddValue ("areaXMin", "Minimum X coordinate of simulation area", areaXMin);
  cmd.AddValue ("areaXMax", "Maximum X coordinate of simulation area", areaXMax);
  cmd.AddValue ("areaYMin", "Minimum Y coordinate of simulation area", areaYMin);
  cmd.AddValue ("areaYMax", "Maximum Y coordinate of simulation area", areaYMax);
  cmd.Parse (argc, argv);

  // Enable logging (optional - comment out for less output)
  // LogComponentEnable ("HandoverSimulation", LOG_LEVEL_INFO);
  // LogComponentEnable ("LteUeRrc", LOG_LEVEL_INFO);
  // LogComponentEnable ("LteEnbRrc", LOG_LEVEL_INFO);

  // Create logger
  g_logger = CreateObject<HandoverDataLogger> ();
  g_logger->Initialize ("handover_dataset.csv");

  // Create the simulation
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Set pathloss model (3GPP UMa model for urban macro cell)
  // Using Friis propagation loss model (more stable for basic simulations)
  // For more realistic results, use: ns3::HybridBuildingsPropagationLossModel
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));

  // Set scheduler
  lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");

  // Configure interference (downlink and uplink as per paper)
  // Downlink and Uplink interference are enabled by default in ns3 LTE module
  // The interference is handled through the channel model and scheduler
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (numBands)); // 50 bands
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (numBands)); // 50 bands
  
  // Enable interference explicitly (ns3 LTE module handles this automatically)
  // Downlink interference is modeled through inter-cell interference
  // Uplink interference is modeled through multiple UEs transmitting simultaneously
  if (downlinkInterference)
    {
      // Downlink interference is automatically enabled in ns3 LTE module
      // through the use of Inter-cell interference modeling
      NS_LOG_INFO ("Downlink interference: ENABLED");
    }
  
  if (uplinkInterference)
    {
      // Uplink interference is automatically enabled in ns3 LTE module
      NS_LOG_INFO ("Uplink interference: ENABLED");
    }

  // Set UE and eNB transmission power
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (gnbTxPower));
  
  // Configure BLER parameters (Target BLER and BLER shift)
  // These parameters affect the AMC (Adaptive Modulation and Coding) module
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (targetBler)); // Target BLER
  // Note: BLER shift and feedback period are typically handled by the AMC and HARQ modules
  
  // Configure carrier frequency
  // In ns3 LTE, carrier frequency is typically set per eNB device
  // We'll set it when installing eNB devices

  // Enable Fading (optional - comment out if fading traces are not available)
  // lteHelper->SetFadingModel ("ns3::TraceFadingLossModel");
  // std::ifstream ifTraceFile;
  // ifTraceFile.open ("../../../src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad", std::ifstream::in);
  // if (ifTraceFile.good ())
  //   {
  //     lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("../../../src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
  //   }

  // Create nodes
  NodeContainer gnbNodes;
  NodeContainer ueNodes;
  gnbNodes.Create (numGnbs);
  ueNodes.Create (numUes);

  // Create mobility model for gNBs (fixed positions)
  Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator> ();
  
  // Predefined gNB positions (distributed in the area to provide coverage)
  // Grid-like deployment: 3 columns, 3 rows (8 gNBs total)
  double xStep = (areaXMax - areaXMin) / 3.0;
  double yStep = (areaYMax - areaYMin) / 2.0;
  
  // Position gNBs in a grid
  uint16_t gnbIndex = 0;
  for (uint16_t row = 0; row < 3 && gnbIndex < numGnbs; ++row)
    {
      for (uint16_t col = 0; col < 3 && gnbIndex < numGnbs; ++col)
        {
          double x = areaXMin + (col + 1) * xStep;
          double y = areaYMin + (row + 1) * yStep;
          gnbPositionAlloc->Add (Vector (x, y, 30.0)); // 30m height
          gnbIndex++;
        }
    }

  MobilityHelper gnbMobility;
  gnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  gnbMobility.SetPositionAllocator (gnbPositionAlloc);
  gnbMobility.Install (gnbNodes);

  // Create mobility model for UEs
  MobilityHelper ueMobility;
  
  // Check if SUMO TCL trace file is provided
  bool useSumoTrace = !sumoTraceFile.empty ();
  
  if (useSumo || useSumoTrace)
    {
      if (useSumoTrace)
        {
          // Use SUMO TCL trace file with Ns2MobilityHelper
          NS_LOG_INFO ("Loading SUMO mobility trace from: " << sumoTraceFile);
          
          // Verify trace file exists
          std::ifstream traceFileCheck (sumoTraceFile.c_str ());
          if (!traceFileCheck.good ())
            {
              NS_FATAL_ERROR ("SUMO trace file not found: " << sumoTraceFile 
                              << ". Please generate it using sumo_to_ns3_trace.py or sumo_helper.py");
            }
          traceFileCheck.close ();
          
          // Ns2MobilityHelper parses ns2-style TCL trace files
          // The trace file should contain commands like:
          //   $node_(0) set X_ 400.00
          //   $node_(0) set Y_ 300.00
          //   $node_(0) set Z_ 1.5
          //   $ns_ at 0.10 "$node_(0) setdest 402.55 300.00 25.50"
          
          // Create Ns2MobilityHelper with the trace file
          // Ns2MobilityHelper parses ns2-style TCL trace files and installs WaypointMobilityModel
          Ns2MobilityHelper ns2MobilityHelper (sumoTraceFile);
          
          // Install the trace on UE nodes
          // This will apply mobility from the trace file to nodes 0 to (numUes-1)
          // Note: The trace file must have nodes numbered starting from 0
          // Nodes in trace file: $node_(0), $node_(1), ..., $node_(N-1) where N = numUes
          ns2MobilityHelper.Install (ueNodes.Begin (), ueNodes.End ());
          
          NS_LOG_INFO ("SUMO TCL trace loaded successfully for " << numUes << " UEs");
          
          // Verify SUMO trace coordinates match simulation area
          // Check initial positions from trace (they should be within the simulation area)
          // Note: This is a basic check - full verification would require parsing the entire trace file
          NS_LOG_INFO ("Simulation area: X[" << areaXMin << "," << areaXMax << "] Y[" << areaYMin << "," << areaYMax << "]");
          NS_LOG_INFO ("Note: SUMO trace coordinates should match this area for proper simulation");
          NS_LOG_INFO ("If positions are outside this range, gNB coverage may not reach UEs");
        }
      else if (useSumo)
        {
          // Note: Real-time SUMO integration requires additional ns3 modules
          // For now, we use RandomWaypoint as fallback
          // If you have SUMO integration module installed, uncomment below:
          /*
          ueMobility.SetMobilityModel ("ns3::SumoMobilityModel",
                                        "CommandLine", StringValue (sumoBinary),
                                        "ConfigFile", StringValue (sumoConfigFile),
                                        "StartTime", DoubleValue (0.0),
                                        "StopTime", DoubleValue (simTime + 1.0));
          */
          NS_LOG_WARN ("SUMO real-time integration not available. Use --sumoTrace option with TCL file.");
          NS_LOG_WARN ("Falling back to RandomWaypoint mobility.");
          useSumo = false;
        }
    }
  
  if (!useSumo && !useSumoTrace)
    {
      // RandomWaypoint mobility (as specified in paper)
      // Speed: uniform distribution between 10 m/s and 60 m/s
      Ptr<RandomRectanglePositionAllocator> positionAlloc = CreateObject<RandomRectanglePositionAllocator> ();
      Ptr<UniformRandomVariable> xVar = CreateObject<UniformRandomVariable> ();
      xVar->SetAttribute ("Min", DoubleValue (areaXMin));
      xVar->SetAttribute ("Max", DoubleValue (areaXMax));
      Ptr<UniformRandomVariable> yVar = CreateObject<UniformRandomVariable> ();
      yVar->SetAttribute ("Min", DoubleValue (areaYMin));
      yVar->SetAttribute ("Max", DoubleValue (areaYMax));
      
      positionAlloc->SetX (xVar);
      positionAlloc->SetY (yVar);
      
      ueMobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue ("ns3::UniformRandomVariable[Min=10.0|Max=60.0]"),
                                    "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
                                    "PositionAllocator", PointerValue (positionAlloc));
      
      // Install RandomWaypoint mobility model
      ueMobility.Install (ueNodes);
    }
  
  // Note: If SUMO trace is used, mobility is already installed above

  // Install LTE devices
  NetDeviceContainer gnbDevs = lteHelper->InstallEnbDevice (gnbNodes);
  
  // CRITICAL: Add X2 interfaces between eNBs for handover support
  // Without X2, handover procedures cannot execute even if measurement reports are received
  lteHelper->AddX2Interface (gnbNodes);
  
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice (ueNodes);
  
  // Configure Cell IDs and Master IDs for eNBs
  // Set MacCellId, MasterId, NrMacCellId, and NrMasterId for each eNB
  for (uint32_t i = 0; i < gnbNodes.GetN (); ++i)
    {
      Ptr<NetDevice> dev = gnbDevs.Get (i);
      Ptr<LteEnbNetDevice> enbDev = dev->GetObject<LteEnbNetDevice> ();
      
      if (enbDev)
        {
          // Set Cell ID (MacCellId)
          if (i == 0)
            {
              enbDev->GetCellId (); // Get default cell ID
              // Cell ID is automatically assigned, but we can verify it
            }
          
          // Note: In ns3 LTE module, Cell IDs are automatically assigned sequentially
          // MasterId and NrMasterId are LTE-specific and may not directly map to ns3's LTE module
          // These are typically used in 5G NR scenarios
        }
    }
  
  // Configure Carrier Aggregation parameters
  // Note: Carrier Aggregation in ns3 requires special configuration
  // For single carrier (numComponentCarriers = 1), standard configuration is sufficient
  if (numComponentCarriers > 1)
    {
      NS_LOG_WARN ("Carrier Aggregation with multiple component carriers requires additional configuration");
      NS_LOG_WARN ("Current implementation uses single carrier (numComponentCarriers = 1)");
    }
  
  // Configure numerology (subcarrier spacing)
  // In ns3 LTE module, numerology affects the subcarrier spacing
  // Numerology 0 = 15 kHz subcarrier spacing (LTE standard)
  if (numerology != 0)
    {
      NS_LOG_WARN ("Numerology other than 0 may require 5G NR module (not standard LTE)");
    }

  // Enable handover with A3 RSRP algorithm
  if (enableHandover)
    {
      lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
      // Configure handover parameters (hysteresis and time-to-trigger)
      // Note: Lower hysteresis and shorter time-to-trigger will cause more frequent handovers
      // Current settings: 3 dB hysteresis (requires 3 dB better signal), 256 ms time-to-trigger
      // These are relatively conservative settings - reducing them will trigger more handovers
      // For more handovers: reduce hysteresis to 1.0-2.0 dB, reduce timeToTrigger to 64-128 ms
      
      lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (handoverHysteresis));
      lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (timeToTriggerMs)));
      
      NS_LOG_INFO ("Handover: ENABLED (A3 RSRP algorithm)");
      NS_LOG_INFO ("  Hysteresis: " << handoverHysteresis << " dB");
      NS_LOG_INFO ("  TimeToTrigger: " << timeToTriggerMs << " ms");
      NS_LOG_INFO ("  Note: Reducing hysteresis or time-to-trigger will increase handover frequency");
    }
  
  // Configure Dynamic Cell Association
  // In ns3 LTE, cell association is dynamic by default when handover is enabled
  if (dynamicCellAssociation)
    {
      NS_LOG_INFO ("Dynamic Cell Association: ENABLED");
      // Dynamic cell association is handled automatically by the handover algorithm
      // UEs will be dynamically associated with the best serving cell based on RSRP measurements
    }
  else
    {
      NS_LOG_INFO ("Dynamic Cell Association: DISABLED (static association)");
    }

  // Install IP stack
  InternetStackHelper internet;
  internet.Install (ueNodes);
  
  // Assign IP addresses
  Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));
  
  // Attach UEs to eNBs
  // If dynamic cell association is enabled, initial attachment is based on signal strength
  // Otherwise, use round-robin assignment
  for (uint16_t i = 0; i < numUes; ++i)
    {
      Ptr<Node> ueNode = ueNodes.Get (i);
      Ptr<NetDevice> ueDev = ueDevs.Get (i);
      
      if (dynamicCellAssociation)
        {
          // With dynamic cell association, UEs will select the best serving cell
          // based on RSRP measurements. For initial attachment, we attach to the
          // closest eNB (based on distance), and then handover will handle dynamic
          // re-association as the UE moves.
          // Find the closest eNB for initial attachment
          Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel> ();
          Vector uePos = ueMobility->GetPosition ();
          
          uint32_t closestEnb = 0;
          double minDistance = std::numeric_limits<double>::max ();
          
          for (uint32_t j = 0; j < gnbNodes.GetN (); ++j)
            {
              Ptr<MobilityModel> enbMobility = gnbNodes.Get (j)->GetObject<MobilityModel> ();
              Vector enbPos = enbMobility->GetPosition ();
              double distance = std::sqrt (std::pow (uePos.x - enbPos.x, 2) + 
                                          std::pow (uePos.y - enbPos.y, 2));
              if (distance < minDistance)
                {
                  minDistance = distance;
                  closestEnb = j;
                }
            }
          lteHelper->Attach (ueDev, gnbDevs.Get (closestEnb));
        }
      else
        {
          // Static association: round-robin assignment
          lteHelper->Attach (ueDev, gnbDevs.Get (i % numGnbs));
        }
      
      // Note: When EPC is used (as in this simulation), data radio bearers are
      // automatically activated by the EPC when UEs attach. We should NOT call
      // ActivateDataRadioBearer manually when EPC is enabled.
      // 
      // If EPC was not being used, we would activate bearers like this:
      // enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
      // EpsBearer bearer (q);
      // lteHelper->ActivateDataRadioBearer (ueDev, bearer);
    }

  // Create remote host for traffic
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internetRemote;
  internetRemote.Install (remoteHostContainer);

  // Create Internet link
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  
  // Routing
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create CBR traffic applications (Constant Bit Rate as per paper)
  uint16_t dlPort = 1234;
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      ++ulPort;
      ++dlPort;
      
      // Downlink: Remote host to UE
      PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                           InetSocketAddress (Ipv4Address::GetAny (), dlPort));
      serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));

      OnOffHelper dlClientHelper ("ns3::UdpSocketFactory",
                                  InetSocketAddress (ueIpIfaces.GetAddress (u), dlPort));
      dlClientHelper.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
      dlClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
      dlClientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      dlClientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      clientApps.Add (dlClientHelper.Install (remoteHost));

      // Uplink: UE to Remote host
      PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                           InetSocketAddress (Ipv4Address::GetAny (), ulPort));
      serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

      OnOffHelper ulClientHelper ("ns3::UdpSocketFactory",
                                  InetSocketAddress (internetIpIfaces.GetAddress (1), ulPort));
      ulClientHelper.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
      ulClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
      ulClientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      ulClientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      clientApps.Add (ulClientHelper.Install (ueNodes.Get (u)));
    }

  serverApps.Start (Seconds (0.01));
  clientApps.Start (Seconds (0.01));
  serverApps.Stop (Seconds (simTime));
  clientApps.Stop (Seconds (simTime));

  // Connect handover traces
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  
  // Connect PHY trace for real RSRP and SINR values (more accurate than RRC reports)
  // This is critical for getting real SINR values instead of placeholders
  Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
                   MakeCallback (&UePhyRsrpSinr));
  
  // Connect measurement reports - eNB side receives RSRQ from UE measurement reports
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                   MakeCallback (&NotifyRecvMeasurementReport));
  
  // UE side: Alternative if available (may not work in all NS3 versions)
  // Try to connect to UE measurement trace (uncomment if the path exists)
  // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ReportCurrentCellRsrpMeasurements",
  //                  MakeCallback (&NotifyReportUeMeasurements));

  // Start periodic logging exactly on whole seconds (starting at 1.0 second)
  Simulator::Schedule (Seconds (1.0), &LogEverySecond, ueNodes, ueDevs);

  // Install FlowMonitor for throughput statistics
  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> monitor = flowHelper.InstallAll ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
  
  // Start periodic throughput sampling (every 0.5 seconds)
  // This updates throughput maps during simulation so they're available for logging
  // Start after applications have started (1 second delay to allow flows to establish)
  Simulator::Schedule (Seconds (1.0), &SampleThroughput, monitor, classifier, ueDevs, ueIpIfaces, numUes, 0.5);

  // Enable PCAP tracing (optional - comment out to disable)
  // p2ph.EnablePcapAll ("handover-simulation");

  // Run simulation
  Simulator::Stop (Seconds (simTime));
  NS_LOG_UNCOND ("Starting simulation...");
  NS_LOG_UNCOND ("Simulation parameters:");
  NS_LOG_UNCOND ("  Number of UEs: " << numUes);
  NS_LOG_UNCOND ("  Number of gNBs: " << numGnbs);
  NS_LOG_UNCOND ("  Simulation time: " << simTime << " seconds");
  NS_LOG_UNCOND ("  UE Tx Power: " << ueTxPower << " dBm");
  NS_LOG_UNCOND ("  gNB Tx Power: " << gnbTxPower << " dBm");
  NS_LOG_UNCOND ("  Carrier Frequency: " << carrierFreq / 1e9 << " GHz");
  NS_LOG_UNCOND ("  Area: X[" << areaXMin << "," << areaXMax << "] Y[" << areaYMin << "," << areaYMax << "]");
  NS_LOG_UNCOND ("  Target BLER: " << targetBler);
  NS_LOG_UNCOND ("  BLER Shift: " << blerShift);
  NS_LOG_UNCOND ("  Feedback Period: " << fbPeriod);
  NS_LOG_UNCOND ("  Number of Component Carriers: " << numComponentCarriers);
  NS_LOG_UNCOND ("  Numerology: " << numerology);
  NS_LOG_UNCOND ("  Number of Bands: " << numBands);
  NS_LOG_UNCOND ("  MacCellId: " << macCellId);
  NS_LOG_UNCOND ("  MasterId: " << masterId);
  NS_LOG_UNCOND ("  NrMacCellId: " << nrMacCellId);
  NS_LOG_UNCOND ("  NrMasterId: " << nrMasterId);
  NS_LOG_UNCOND ("  Dynamic Cell Association: " << (dynamicCellAssociation ? "true" : "false"));
  NS_LOG_UNCOND ("  Handover: " << (enableHandover ? "true" : "false"));
  NS_LOG_UNCOND ("  Downlink Interference: " << (downlinkInterference ? "true" : "false"));
  NS_LOG_UNCOND ("  Uplink Interference: " << (uplinkInterference ? "true" : "false"));
  
  Simulator::Run ();

  // Collect statistics (classifier was already created above for periodic sampling)
  monitor->CheckForLostPackets ();
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

  // Extract throughput per UE from FlowMonitor
  // Note: Throughput is calculated as average over entire simulation
  // For real-time throughput, we would need periodic tracking, but this gives overall average
  
  // Map IP addresses to IMSI for throughput tracking
  std::map<Ipv4Address, uint64_t> ipToImsi; // IP -> IMSI mapping
  for (uint16_t i = 0; i < numUes; ++i)
    {
      if (i < ueIpIfaces.GetN ())
        {
          Ipv4Address ueIp = ueIpIfaces.GetAddress (i);
          Ptr<NetDevice> ueDev = ueDevs.Get (i);
          Ptr<LteUeNetDevice> lteUeDev = ueDev->GetObject<LteUeNetDevice> ();
          if (lteUeDev)
            {
              uint64_t imsi = lteUeDev->GetImsi ();
              ipToImsi[ueIp] = imsi;
            }
        }
    }
  
  // Calculate throughput per UE from flow statistics
  // Sum throughput from all flows per UE (each UE may have multiple flows)
  std::map<uint64_t, double> totalThroughputDl; // IMSI -> total DL throughput
  std::map<uint64_t, double> totalThroughputUl; // IMSI -> total UL throughput
  
  for (auto it = stats.begin (); it != stats.end (); ++it)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (it->first);
      
      if (simTime > 0)
        {
          double flowThroughputDl = it->second.rxBytes * 8.0 / (simTime * 1000000.0); // Mbps
          double flowThroughputUl = it->second.txBytes * 8.0 / (simTime * 1000000.0); // Mbps
          
          // Downlink: packets to UE IP (from remote host)
          if (ipToImsi.find (t.destinationAddress) != ipToImsi.end ())
            {
              uint64_t imsi = ipToImsi[t.destinationAddress];
              totalThroughputDl[imsi] += flowThroughputDl;
            }
          
          // Uplink: packets from UE IP (to remote host)
          if (ipToImsi.find (t.sourceAddress) != ipToImsi.end ())
            {
              uint64_t imsi = ipToImsi[t.sourceAddress];
              totalThroughputUl[imsi] += flowThroughputUl;
            }
        }
    }
  
  // Store final throughput values
  for (auto it = totalThroughputDl.begin (); it != totalThroughputDl.end (); ++it)
    {
      ueThroughputDl[it->first] = it->second;
    }
  for (auto it = totalThroughputUl.begin (); it != totalThroughputUl.end (); ++it)
    {
      ueThroughputUl[it->first] = it->second;
    }
  
  NS_LOG_INFO ("Throughput extraction completed for " << ueThroughputDl.size () << " UEs");

  // Log flow statistics to CSV
  std::ofstream flowStatsFile ("flow_statistics.csv");
  flowStatsFile << "Flow_ID,Source,Destination,Throughput_DL_Mbps,Throughput_UL_Mbps,"
                << "Packets_Sent,Packets_Received,Packets_Lost,Delay_Mean_ms,Jitter_ms\n";

  for (auto it = stats.begin (); it != stats.end (); ++it)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (it->first);
      double throughputDl = 0.0;
      double throughputUl = 0.0;
      double delayMean = 0.0;
      double jitterMean = 0.0;
      
      if (simTime > 0)
        {
          throughputDl = it->second.rxBytes * 8.0 / (simTime * 1000000.0); // Mbps
          throughputUl = it->second.txBytes * 8.0 / (simTime * 1000000.0); // Mbps
        }
      
      if (it->second.rxPackets > 0)
        {
          delayMean = it->second.delaySum.GetSeconds () / it->second.rxPackets * 1000.0; // ms
          jitterMean = it->second.jitterSum.GetSeconds () / (it->second.rxPackets - 1) * 1000.0; // ms
        }
      
      flowStatsFile << it->first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                    << throughputDl << "," << throughputUl << ","
                    << it->second.txPackets << "," << it->second.rxPackets << ","
                    << it->second.lostPackets << "," << delayMean << "," << jitterMean << "\n";
    }
  flowStatsFile.close ();

  // Cleanup
  Simulator::Destroy ();
  
  NS_LOG_UNCOND ("Simulation completed!");
  NS_LOG_UNCOND ("Data exported to:");
  NS_LOG_UNCOND ("  - handover_dataset.csv (handover events and measurements)");
  NS_LOG_UNCOND ("  - flow_statistics.csv (traffic statistics)");
  
  return 0;
}

