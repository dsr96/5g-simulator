/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2024 University of Malaga (UMA)
 *   Author: David Segura Ramos
 *
 */

#include "ns3/core-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/nr-module.h"
#include "ns3/config-store-module.h"
#include "ns3/lte-module.h"
#include "ns3/dualsocket5gapp-module.h"
#include "ns3/dualsocketapp-module.h"
#include <ns3/buildings-module.h>
#include "ns3/random-variable-stream.h"
#include "ns3/antenna-module.h"

#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>

/**
  *
  * To run the simulation with the default configuration one shall run the
  * following in the command line:
  *
  * ./waf --run scratch/dual-connectivity
  *
  */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("dual-connectivity");

void
TraceBuildingLoc ()
{
  //Write the location of buildings.
  static bool firstTime = true;
  if (firstTime)
    {
      firstTime = false;
      std::ofstream outFile;
      outFile.open ("building.txt");
      if (!outFile.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file building.txt");
        }
      for (BuildingList::Iterator bit = BuildingList::Begin (); bit != BuildingList::End (); ++bit)
        {
          Box boundaries = (*bit)->GetBoundaries ();
          outFile << boundaries.xMin << "\t";
          outFile << boundaries.xMax << "\t";
          outFile << boundaries.yMin << "\t";
          outFile << boundaries.yMax << "\t";
          outFile << boundaries.zMin << "\t";
          outFile << boundaries.zMax << std::endl;
        }
      outFile.close ();
    }
}

std::ofstream outFileRxPacket;

void
RxPacketServer (std::string path, const Ptr<const Packet> packet, const Address &from,
                const Address &dest)
{
  outFileRxPacket << "At time " << Simulator::Now ().GetSeconds () << " packet in server received "
                  << packet->GetSize () << " bytes from "
                  << InetSocketAddress::ConvertFrom (from).GetIpv4 () << " port "
                  << InetSocketAddress::ConvertFrom (from).GetPort () << " to "
                  << InetSocketAddress::ConvertFrom (dest).GetIpv4 () << " port "
                  << InetSocketAddress::ConvertFrom (dest).GetPort () << std::endl;
}

struct timePositionStruct
{
  Time m_time;
  uint32_t m_node_id;
  Vector m_pos;
};
std::vector<timePositionStruct> g_timeposition;
std::ofstream m_courseChangeFile;

static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  /*timePositionStruct tms;
	 tms.m_time = Simulator::Now();
	 tms.m_node_id = mobility->GetObject<Node> ()->GetId ();
	 tms.m_pos = mobility->GetPosition (); // Get position
	 g_timeposition.push_back(tms);*/

  //std::cout<<"Time: "<< tms.m_time.GetSeconds() <<" "<< tms.m_node_id << " POS: x=" << tms.m_pos.x << ", y=" << tms.m_pos.y<< ", z=" << tms.m_pos.z << std::endl;
  std::cout << "Course change: " << std::endl;
  std::cout << "Time: " << Simulator::Now ().GetSeconds () << std::endl;
  std::cout << "NodeId: " << mobility->GetObject<Node> ()->GetId () << std::endl;
  std::cout << "Position: " << mobility->GetPosition ().x << " " << mobility->GetPosition ().y
            << " " << mobility->GetPosition ().z << std::endl;
  std::cout << std::endl;
  m_courseChangeFile << Simulator::Now ().GetSeconds () << "\t";
  m_courseChangeFile << mobility->GetObject<Node> ()->GetId () << "\t";
  m_courseChangeFile << mobility->GetPosition ().x << "\t";
  m_courseChangeFile << mobility->GetPosition ().y << "\t";
  m_courseChangeFile << mobility->GetPosition ().z << std::endl;
}

void
SaveTimePositions (std::string filename, const std::vector<timePositionStruct> &events)
{
  //std::string filename = "timePositions.txt",
  std::ofstream outFile;
  //outFile.open (filename.c_str (), std::ofstream::out | std::ofstream::app);
  outFile.open (filename.c_str ());
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  outFile.setf (std::ios_base::fixed);

  outFile << "Time\tNodeID\tPosX\tPosY\tPosZ" << std::endl;

  for (std::vector<timePositionStruct>::size_type i = 0; i < events.size (); i++)
    {
      outFile << events[i].m_time.GetSeconds () << "\t";
      outFile << events[i].m_node_id << "\t";
      outFile << events[i].m_pos.x << "\t";
      outFile << events[i].m_pos.y << "\t";
      outFile << events[i].m_pos.z << std::endl;
    }
  outFile.close ();
}

std::ofstream m_outSinrFile; //!< the output file stream for the SINR file
std::ofstream m_outSnrFile; //!< the output file stream for the SNR file
std::ofstream m_outRssiFile; //!< the output file stream for the RSSI file

std::ofstream m_outSinrFileGnb; //!< the output file stream for the SINR file
std::ofstream m_outSnrFileGnb; //!< the output file stream for the SNR file
std::ofstream m_outRssiFileGnb; //!< the output file stream for the RSSI file

std::ofstream m_outSinrFileGnb1; //!< the output file stream for the SINR file
std::ofstream m_outSnrFileGnb1; //!< the output file stream for the SNR file
std::ofstream m_outRssiFileGnb1; //!< the output file stream for the RSSI file

struct ValueMap
{
  uint64_t sourceCellId;
  uint16_t sourceRnti;
  uint64_t targetCellId;
  uint16_t targetRnti;
  double sourceSinr;
  double targetSinr;
  uint8_t sourceCqi;
  uint8_t targetCqi;
};

std::unordered_map<std::string, ValueMap> umap;

void
UeReception (std::string path, RxPacketTraceParams params)
{
  std::size_t tam = path.find ('/', 11);
  std::string nodeId = path.substr (10, tam - 10);
  std::size_t tam2 = path.find ("/", tam + 12);
  std::string deviceId = path.substr (tam + 12, tam2 - (tam + 12));

  if (params.m_corrupt == 0)
    {
      //std::cout << path << std::endl;
      //std::cout << "NODE: " << nodeId << std::endl;
      //std::cout << "DEVICE: " << deviceId << std::endl;
      ValueMap value;

      if (umap.find (nodeId) == umap.end ())
        {
          //not find, insert value
          if (deviceId == "0")
            {
              value.sourceCellId = params.m_cellId + 1;
              value.sourceRnti = params.m_rnti;
              value.sourceSinr = 10 * log10 (params.m_sinr);
            }
          else //deviceId = 1
            {
              value.targetCellId = params.m_cellId + 1;
              value.targetRnti = params.m_rnti;
              value.targetSinr = 10 * log10 (params.m_sinr);
            }
          umap.insert (std::make_pair (nodeId, value));
        }
      else
        {
          value = umap[nodeId];
          //update value
          if (deviceId == "0")
            {
              value.sourceCellId = params.m_cellId + 1;
              value.sourceRnti = params.m_rnti;
              value.sourceSinr = 10 * log10 (params.m_sinr);
            }
          else //deviceId = 1
            {
              value.targetCellId = params.m_cellId + 1;
              value.targetRnti = params.m_rnti;
              value.targetSinr = 10 * log10 (params.m_sinr);
            }
          umap[nodeId] = value;
        }
    }
  m_outSinrFile << params.m_cellId << params.m_rnti << "\t" << 10 * log10 (params.m_sinr)
                << std::endl;
}

void
UeSnrPerProcessedChunk (double snr)
{
  m_outSnrFile << Simulator::Now ().GetNanoSeconds () << "\t" << 10 * log10 (snr) << std::endl;
}

void
UeRssiPerProcessedChunk (double rssidBm)
{
  m_outRssiFile << Simulator::Now ().GetNanoSeconds () << "\t" << rssidBm << std::endl;
}

void
GnbReception (RxPacketTraceParams params)
{
  m_outSinrFileGnb << Simulator::Now ().GetNanoSeconds () << "\t" << params.m_cellId << "\t"
                   << params.m_rnti << "\t" << 10 * log10 (params.m_sinr) << std::endl;
}

void
GnbSnrPerProcessedChunk (double snr)
{
  m_outSnrFileGnb << Simulator::Now ().GetNanoSeconds () << "\t" << 10 * log10 (snr) << std::endl;
}

void
GnbRssiPerProcessedChunk (double rssidBm)
{
  m_outRssiFileGnb << Simulator::Now ().GetNanoSeconds () << "\t" << rssidBm << std::endl;
}

void
Gnb1Reception (RxPacketTraceParams params)
{
  m_outSinrFileGnb1 << Simulator::Now ().GetNanoSeconds () << "\t" << params.m_cellId << "\t"
                    << params.m_rnti << "\t" << 10 * log10 (params.m_sinr) << std::endl;
}

void
Gnb1SnrPerProcessedChunk (double snr)
{
  m_outSnrFileGnb1 << Simulator::Now ().GetNanoSeconds () << "\t" << 10 * log10 (snr) << std::endl;
}

void
Gnb1RssiPerProcessedChunk (double rssidBm)
{
  m_outRssiFileGnb1 << Simulator::Now ().GetNanoSeconds () << "\t" << rssidBm << std::endl;
}

void
ReportSlotStatsNr (const SfnSf &sfnSf, uint32_t scheduledUe, uint32_t usedReg, uint32_t usedSym,
                   uint32_t availableRb, uint32_t availableSym, uint16_t bwpId, uint16_t cellId)
{
  std::cout << "FRAME: " << (uint32_t) sfnSf.GetFrame () << std::endl;
  std::cout << "SUBFRAME: " << unsigned ((uint8_t) sfnSf.GetSubframe ()) << std::endl;
  std::cout << "SLOT: " << (uint16_t) sfnSf.GetSlot () << std::endl;
  std::cout << "SCHEDULED UE: " << scheduledUe << std::endl;
  std::cout << "USED RE: " << usedReg << std::endl;
  std::cout << "USED SYMBOLS: " << usedSym << std::endl;
  std::cout << "AVAILABLE RB: " << availableRb << std::endl;
  std::cout << "AVAILABLE SYMBOLS: " << availableSym << std::endl;
  std::cout << "####################################" << std::endl;
}

void
ActivatePacketDuplication (Ptr<LteEnbRrc> masterGnbRrc, Ptr<NetDevice> ueNetDev,
                           uint16_t sourceCellId, uint16_t targetCellId,
                           Ptr<NetDevice> ueNetDevDest)
{
  uint16_t sourceRnti = ueNetDev->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ();
  uint16_t targetRnti = ueNetDevDest->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ();
  std::cout << "SOURCE RNTI: " << sourceRnti << std::endl;
  std::cout << "TARGET RNTI: " << targetRnti << std::endl;
  masterGnbRrc->ActivatePacketDuplication (sourceRnti, sourceCellId, targetCellId, targetRnti);
}

void
DeActivatePacketDuplication (Ptr<LteEnbRrc> masterGnbRrc, Ptr<NetDevice> ueNetDev)
{
  uint16_t sourceRnti = ueNetDev->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ();
  masterGnbRrc->DeActivatePacketDuplication (sourceRnti);
}

void
ReportCQI (std::string path, uint16_t rnti, uint16_t cellId, uint8_t cqi)
{
  std::size_t tam = path.find ('/', 11);
  std::string nodeId = path.substr (10, tam - 10);
  std::size_t tam2 = path.find ("/", tam + 12);
  std::string deviceId = path.substr (tam + 12, tam2 - (tam + 12));

  //std::cout << path << std::endl;
  std::cout << "NODE: " << nodeId << std::endl;
  std::cout << "DEVICE: " << deviceId << std::endl;
  std::cout << "UE RNTI: " << rnti << std::endl;
  std::cout << "CELL ID: " << cellId << std::endl;
  std::cout << "CQI value: " << unsigned (cqi) << std::endl;
  std::cout << std::endl;

  //Add value to the map
  ValueMap value;

  if (umap.find (nodeId) == umap.end ())
    {
      //not find, insert value
      if (deviceId == "0")
        {
          value.sourceCellId = cellId;
          value.sourceRnti = rnti;
          value.sourceCqi = cqi;
        }
      else //deviceId = 1
        {
          value.targetCellId = cellId;
          value.targetRnti = rnti;
          value.targetCqi = cqi;
        }
      umap.insert (std::make_pair (nodeId, value));
    }
  else
    {
      value = umap[nodeId];
      //update value
      if (deviceId == "0")
        {
          value.sourceCellId = cellId;
          value.sourceRnti = rnti;
          value.sourceCqi = cqi;
        }
      else //deviceId = 1
        {
          value.targetCellId = cellId;
          value.targetRnti = rnti;
          value.targetCqi = cqi;
        }
      umap[nodeId] = value;
    }
}

void
ReportRSRP (std::string path, double power, uint16_t cellId, uint16_t rnti)
{
  std::cout << "RNTI " << rnti << " CELLID: " << cellId << " RSRP (dBm): " << power << std::endl;
}

EventId m_algoritmEvent;

void
EvaluateInputs (Ptr<LteEnbRrc> masterGnbRrc) //, uint16_t sourceCellId, uint16_t targetCellId)
{

  std::unordered_map<std::string, ValueMap>::iterator itr;
  //static bool activate = false;
  bool activate = false;

  for (itr = umap.begin (); itr != umap.end (); itr++)
    {
      // itr works as a pointer to pair<string, double>
      // type itr->first stores the key part  and
      // itr->second stroes the value part
      std::cout << itr->first << "  " << itr->second.sourceSinr << "  " << itr->second.sourceRnti
                << "  " << itr->second.sourceCellId << "  " << itr->second.targetSinr << "  "
                << itr->second.targetRnti << "  " << itr->second.targetCellId
                << " SOURCE CQI: " << unsigned (itr->second.sourceCqi)
                << " TARGET CQI: " << unsigned (itr->second.targetCqi) << std::endl;

      double sinrThreshold = 20;
      if ((itr->second.sourceSinr < sinrThreshold) && (itr->second.sourceCqi < 12))
        {
          activate = true;
        }

      if (activate == true)
        {
          //masterGnbRrc->ActivatePacketDuplication (itr->second.sourceRnti, itr->second.sourceCellId, itr->second.targetCellId, itr->second.targetRnti);
          activate = true;
        }
      else
        {
          //masterGnbRrc->DeActivatePacketDuplication (itr->second.sourceRnti);
        }
    }

  m_algoritmEvent = Simulator::Schedule (MilliSeconds (500), &EvaluateInputs,
                                         masterGnbRrc); //, sourceCellId, targetCellId);
}

/*void decreaseGnbTxPower ()
{
	nrHelper->GetGnbPhy (gNbNetDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (-42.0));
}

void increaseGnbTxPower ()
{
	nrHelper->GetGnbPhy (gNbNetDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (45.0));
} */

int
main (int argc, char *argv[])
{

  //Packet::EnablePrinting ();
  //Packet::EnableChecking ();

  // enable logging or not
  bool logging = false;
  if (logging)
    {
      LogComponentEnable ("UdpClient", LOG_LEVEL_ALL);
    }

  // set simulation time and mobility
  double simTime = 2000; // seconds
  double udpAppStartTime = 1; //seconds
  double udpInterval = 10; // ms

  //other simulation parameters default values
  uint16_t numerology = 2;
  uint16_t symbolsPerSlot = 14;

  uint16_t gNbNum = 2;

  double centralFrequency = 3.7e9;
  double bandwidth = 20e6;
  double txPower = 23; //dBm
  double txPowerUE = 10; //dBm
  double lambda = 1000;
  uint32_t udpPacketSize = 64;
  uint8_t fixedMcs = 20;
  bool useFixedMcs = false;
  bool singleUeTopology = true;
  // Where we will store the output files.
  std::string simTag = "default";
  std::string outputDir = "./";
  double sinrThreshold = 5.0; // (dB)
  bool enableBuildings = false;
  std::string industrialscenario = "InF-DH";
  bool shannonModel = false;
  double lowClutterDensity = 0.2;
  double highClutterDensity = 0.6;
  double InFVolume = 60000; //80x60x10 m3;
  double InFTotalSurface = 6000; //80x60 m2;
  uint16_t ueNumStaticGnb1 = 15;
  uint16_t ueNumStaticGnb2 = 15;
  bool harqEnabled = true;

  //Blockage
  bool blockage = false;
  int numnonselfblocking = 4;
  bool portraitmode = true;
  double blockerspeed = 1; //m/s

  bool BuildingsEnabled = false;
  bool enableShadowing = true;

  double clutterHeight = 6; //meters

  bool useREM = false;
  uint8_t harqProcesses = 20;

  //Beamforming
  std::string m_beamforming = "REALISTIC";

  CommandLine cmd;
  cmd.AddValue ("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
  cmd.AddValue ("numerology", "The numerology to be used.", numerology);
  cmd.AddValue ("txPower", "Tx power to be configured to gNB", txPower);
  cmd.AddValue ("simTag",
                "tag to be appended to output filenames to distinguish simulation campaigns",
                simTag);
  cmd.AddValue ("outputDir", "directory where to store simulation results", outputDir);
  cmd.AddValue ("frequency", "The system frequency", centralFrequency);
  cmd.AddValue ("bandwidth", "The system bandwidth", bandwidth);
  cmd.AddValue ("udpPacketSize", "UDP packet size in bytes", udpPacketSize);
  cmd.AddValue ("lambda", "Number of UDP packets per second", lambda);
  cmd.AddValue (
      "fixedMcs",
      "The fixed MCS that will be used in this example if useFixedMcs is configured to true (1).",
      fixedMcs);
  cmd.AddValue ("useFixedMcs", "Whether to use fixed mcs, normally used for testing purposes",
                useFixedMcs);
  cmd.AddValue ("singleUeTopology",
                "If true, the example uses a predefined topology with one UE and one gNB; "
                "if false, the example creates a grid of gNBs with a number of UEs attached",
                singleUeTopology);
  cmd.AddValue ("sinrThreshold",
                "SINR (dB) threshold value in gNB in order to activate packet duplication",
                sinrThreshold);
  cmd.AddValue ("txPowerUE", "UE tx power in dBm", txPowerUE);
  cmd.AddValue ("symbolsPerSlot", "Number of symbols per slot", symbolsPerSlot);
  cmd.AddValue ("industrialscenario", "Select InF-SH or InF-DH", industrialscenario);
  cmd.AddValue ("lowClutterDensity",
                "Clutter density (r)- percentage of surface area occupied by clutter (< 0.4). "
                "(InF-SL, InF-SH)",
                lowClutterDensity);
  cmd.AddValue ("highClutterDensity",
                "Clutter density (r)- percentage of surface area occupied by clutter (>= 0.4). "
                "(InF-DL, InF-DH)",
                highClutterDensity);
  cmd.AddValue ("clutterHeight", "Clutter height < 10 m", clutterHeight);
  cmd.AddValue ("InFVolume",
                "Hall volume in m3 in factory scenarios for calculating the delay spread",
                InFVolume);
  cmd.AddValue ("InFTotalSurface",
                "Total surface area of hall in m2 (walls+floor+ceilling) in factory scenarios for "
                "calculating the delay spread",
                InFTotalSurface);
  cmd.AddValue ("shannonModel", "Activate shannon model for AMC", shannonModel);
  cmd.AddValue ("ueNumStaticGnb1",
                "Number of UEs connected to gNB1de UEs conectados al gNB1"
                "full buffer-traffic",
                ueNumStaticGnb1);
  cmd.AddValue ("ueNumStaticGnb2",
                "Number of UEs connected to gNB2"
                "full buffer-traffic",
                ueNumStaticGnb2);
  cmd.AddValue ("blockage", "Enable blockage model A (sec 7.6.4.1)", blockage);
  cmd.AddValue ("numnonselfblocking", "number of non-self-blocking regions", numnonselfblocking);
  cmd.AddValue ("portraitmode", "true for portrait mode, false for landscape mode", portraitmode);
  cmd.AddValue ("blockerspeed", "The speed of moving blockers, the unit is m/s", blockerspeed);
  cmd.AddValue ("buildingsEnabled", "Activate/Deactivate buildings propagation loss model",
                BuildingsEnabled);
  cmd.AddValue ("enableShadowing", "Activate shadowing in propagation loss model", enableShadowing);
  cmd.AddValue ("useREM", "Use REM", useREM);
  cmd.AddValue ("beamformingType", "IDEAL or REALISTIC", m_beamforming);
  cmd.AddValue ("harqEnabled", "Activate HARQ", harqEnabled);
  cmd.AddValue ("harqProcesses", "Number of Harq processes", harqProcesses);
  cmd.AddValue ("simTime", "Simulation Time", simTime);

  cmd.Parse (argc, argv);

  m_outSinrFile.open ("SINR.txt");
  m_outSinrFile.setf (std::ios_base::fixed);

  if (!m_outSinrFile.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SINR.txt");
    }

  m_outSnrFile.open ("SNR.txt");
  m_outSnrFile.setf (std::ios_base::fixed);

  if (!m_outSnrFile.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SNR.txt");
    }

  m_outRssiFile.open ("RSSI.txt");
  m_outRssiFile.setf (std::ios_base::fixed);

  if (!m_outRssiFile.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "RSSI.txt");
    }

  m_outSinrFileGnb.open ("SINR-GNB.txt");
  m_outSinrFileGnb.setf (std::ios_base::fixed);

  if (!m_outSinrFileGnb.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SINR-GNB.txt");
    }

  m_outSnrFileGnb.open ("SNR-GNB.txt");
  m_outSnrFileGnb.setf (std::ios_base::fixed);

  if (!m_outSnrFileGnb.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SNR-GNB.txt");
    }

  m_outRssiFileGnb.open ("RSSI-GNB.txt");
  m_outRssiFileGnb.setf (std::ios_base::fixed);

  if (!m_outRssiFileGnb.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "RSSI-GNB.txt");
    }

  m_outSinrFileGnb1.open ("SINR-GNB1.txt");
  m_outSinrFileGnb1.setf (std::ios_base::fixed);

  if (!m_outSinrFileGnb1.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SINR-GNB1.txt");
    }

  m_outSnrFileGnb1.open ("SNR-GNB1.txt");
  m_outSnrFileGnb1.setf (std::ios_base::fixed);

  if (!m_outSnrFileGnb1.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "SNR-GNB1.txt");
    }

  m_outRssiFileGnb1.open ("RSSI-GNB1.txt");
  m_outRssiFileGnb1.setf (std::ios_base::fixed);

  if (!m_outRssiFileGnb1.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "RSSI-GNB1.txt");
    }

  outFileRxPacket.open ("RxPacket-RemoteHost.txt");
  if (!outFileRxPacket.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "RxPacket-RemoteHost.txt");
    }

  m_courseChangeFile.open ("timePositions.txt");
  if (!m_courseChangeFile.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "timePositions.txt");
    }

  Config::SetDefault ("ns3::EpsBearer::Release", UintegerValue (15));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                      TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::ThreeGppChannelModel::InFTotalSurface", DoubleValue (InFTotalSurface));
  Config::SetDefault ("ns3::ThreeGppChannelModel::Blockage", BooleanValue (blockage));
  Config::SetDefault ("ns3::ThreeGppChannelModel::NumNonselfBlocking",
                      IntegerValue (numnonselfblocking));
  Config::SetDefault ("ns3::ThreeGppChannelModel::PortraitMode", BooleanValue (portraitmode));
  Config::SetDefault ("ns3::ThreeGppChannelModel::BlockerSpeed", DoubleValue (blockerspeed));
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::BuildingsEnabled",
                      BooleanValue (BuildingsEnabled));
  Config::SetDefault ("ns3::NrHelper::HarqEnabled", BooleanValue (harqEnabled));

  //Noise figure (38901 v16.1.0)
  Config::SetDefault ("ns3::NrUePhy::NoiseFigure", DoubleValue (5.0));

  if (industrialscenario == "InF-DH")
    {
      Config::SetDefault ("ns3::ThreeGppIndoorFactoryDHChannelConditionModel::HighClutterDensity",
                          DoubleValue (highClutterDensity));
      Config::SetDefault ("ns3::ThreeGppIndoorFactoryDHChannelConditionModel::ClutterHeight",
                          DoubleValue (clutterHeight));
    }
  else if (industrialscenario == "InF-SH")
    {
      Config::SetDefault ("ns3::ThreeGppIndoorFactorySHChannelConditionModel::LowClutterDensity",
                          DoubleValue (lowClutterDensity));
      Config::SetDefault ("ns3::ThreeGppIndoorFactorySHChannelConditionModel::ClutterHeight",
                          DoubleValue (clutterHeight));
    }

  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay", TimeValue (NanoSeconds (100)));

  // setup the nr simulation
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  nrHelper->SetUeMacAttribute ("NumHarqProcess", UintegerValue (harqProcesses));
  nrHelper->SetGnbMacAttribute ("NumHarqProcess", UintegerValue (harqProcesses));

  /*
   * Spectrum division. We create one operation band with one component carrier
   * (CC) which occupies the whole operation band bandwidth. The CC contains a
   * single Bandwidth Part (BWP). This BWP occupies the whole CC band.
   * Both operational bands will use the StreetCanyon channel modeling.
   */
  double centralFrequencyBand1 = centralFrequency;
  double centralFrequencyBand2 = centralFrequency - 2 * bandwidth;
  double bandwidthBand1 = bandwidth;
  double bandwidthBand2 = bandwidth;

  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1; // in this example, both bands have a single CC
  BandwidthPartInfo::Scenario scenario;

  if (industrialscenario == "InF-DH")
    {
      scenario = BandwidthPartInfo::InF_DH_nLoS;
    }
  else if (industrialscenario == "InF-SH")
    {
      scenario = BandwidthPartInfo::InF_SH;
    }
  else
    {
      scenario = BandwidthPartInfo::InH_OfficeMixed_LoS;
    }

  // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
  // a single BWP per CC
  CcBwpCreator::SimpleOperationBandConf bandConf1 (centralFrequencyBand1, bandwidthBand1,
                                                   numCcPerBand, scenario);
  CcBwpCreator::SimpleOperationBandConf bandConf2 (centralFrequencyBand2, bandwidthBand2,
                                                   numCcPerBand, BandwidthPartInfo::InF_DH_LoS);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf1);
  OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf2);

  /*
   * Initialize channel and pathloss, plus other things inside band1. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&band1);
  nrHelper->InitializeOperationBand (&band2);

  BandwidthPartInfoPtrVector allBwps, bwps1, bwps2;
  allBwps = CcBwpCreator::GetAllBwps ({band1, band2});
  bwps1 = CcBwpCreator::GetAllBwps ({band1});
  bwps2 = CcBwpCreator::GetAllBwps ({band2});

  /////////////////////////////////////////////////////////////////////////////////
  /*
   * Continue setting the parameters which are common to all the nodes, like the
   * gNB transmit power or numerology.
   */
  nrHelper->SetGnbPhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetGnbPhyAttribute ("SymbolsPerSlot", UintegerValue (symbolsPerSlot));
  nrHelper->SetGnbPhyAttribute ("Numerology", UintegerValue (numerology));
  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPowerUE));

  // Scheduler
  nrHelper->SetSchedulerTypeId (TypeId::LookupByName ("ns3::NrMacSchedulerTdmaRR"));
  nrHelper->SetSchedulerAttribute ("FixedMcsDl", BooleanValue (useFixedMcs));
  nrHelper->SetSchedulerAttribute ("FixedMcsUl", BooleanValue (useFixedMcs));
  nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (fixedMcs));
  nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (fixedMcs));

  if (useFixedMcs == true)
    {
      nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (fixedMcs));
      nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (fixedMcs));
    }

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));
  Config::SetDefault ("ns3::LteRlcUm::ReorderingTimer", TimeValue (MilliSeconds (50)));

  // Antennas for all the UEs
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (1));
  nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                   PointerValue (CreateObject<IsotropicAntennaModel> ()));

  // Antennas for all the gNbs
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (1));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (1));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement",
                                    PointerValue (CreateObject<IsotropicAntennaModel> ()));

  // Initialize beamforming
  Ptr<BeamformingHelperBase> beamformingHelper;

  if (m_beamforming == "IDEAL")
    {
      beamformingHelper = CreateObject<IdealBeamformingHelper> ();
      beamformingHelper->SetAttribute ("BeamformingPeriodicity", TimeValue (MilliSeconds (100)));
      beamformingHelper->SetBeamformingMethod (CellScanQuasiOmniBeamforming::GetTypeId ());
      nrHelper->SetSchedulerAttribute ("EnableSrsInUlSlots", BooleanValue (false));
      nrHelper->SetSchedulerAttribute ("EnableSrsInFSlots", BooleanValue (false));
      nrHelper->SetSchedulerAttribute ("SrsSymbols", UintegerValue (1));
    }
  else if (m_beamforming == "REALISTIC")
    {
      beamformingHelper = CreateObject<RealisticBeamformingHelper> ();
      beamformingHelper->SetBeamformingMethod (RealisticBeamformingAlgorithm::GetTypeId ());
      // when realistic beamforming used, also realistic beam manager should be set
      // TODO, move this to NrHelper, so user sets BeamformingMethod calling NrHelper
      nrHelper->SetGnbBeamManagerTypeId (RealisticBfManager::GetTypeId ());
      nrHelper->SetGnbBeamManagerAttribute ("TriggerEvent",
                                            EnumValue (RealisticBfManager::SRS_COUNT));
      nrHelper->SetSchedulerAttribute ("SrsSymbols", UintegerValue (1));
    }
  else
    {
      NS_ABORT_MSG ("Unknown beamforming type.");
    }
  nrHelper->SetBeamformingHelper (beamformingHelper);

  Config::SetDefault ("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue (true));

  nrHelper->SetPathlossAttribute (
      "ShadowingEnabled", BooleanValue (enableShadowing)); //BooleanValue (enableShadowing));

  // Error Model: UE and GNB with same spectrum error model.
  nrHelper->SetUlErrorModel ("ns3::NrEesmIrT1");
  nrHelper->SetDlErrorModel ("ns3::NrEesmIrT1");

  // Both DL and UL AMC will have the same model behind.
  if (shannonModel)
    {
      nrHelper->SetGnbDlAmcAttribute (
          "AmcModel", EnumValue (NrAmc::ShannonModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
      nrHelper->SetGnbUlAmcAttribute (
          "AmcModel", EnumValue (NrAmc::ShannonModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
    }
  else
    {
      nrHelper->SetGnbDlAmcAttribute (
          "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
      nrHelper->SetGnbUlAmcAttribute (
          "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
    }

  // Create EPC helper
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  nrHelper->SetEpcHelper (epcHelper);
  // Core latency
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

  // Initialize nrHelper
  nrHelper->Initialize ();

  /*
   *  Create the gNB and UE nodes according to the network topology
   */
  NodeContainer gNbNodes;
  NodeContainer ueNodes;
  MobilityHelper mobility, mobility2;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> bsPositionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> utPositionAlloc = CreateObject<ListPositionAllocator> ();

  const double gNbHeight = 8;
  const double ueHeight = 1.5;

  uint16_t ueConMobilidad = 1;

  gNbNodes.Create (gNbNum);
  ueNodes.Create (ueConMobilidad + ueNumStaticGnb1 + ueNumStaticGnb2);

  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (0));
  x->SetAttribute ("Max", DoubleValue (20));

  Ptr<UniformRandomVariable> y = CreateObject<UniformRandomVariable> ();
  y->SetAttribute ("Min", DoubleValue (15));
  y->SetAttribute ("Max", DoubleValue (27));

  Ptr<UniformRandomVariable> x2 = CreateObject<UniformRandomVariable> ();
  x2->SetAttribute ("Min", DoubleValue (65));
  x2->SetAttribute ("Max", DoubleValue (75));

  Ptr<UniformRandomVariable> y2 = CreateObject<UniformRandomVariable> ();
  y2->SetAttribute ("Min", DoubleValue (5));
  y2->SetAttribute ("Max", DoubleValue (10));

  for (uint32_t j = 1; j <= ueNumStaticGnb1; ++j)
    {
      utPositionAlloc->Add (Vector (x->GetValue (), y->GetValue (), ueHeight));
    }

  for (uint32_t j = 1; j <= ueNumStaticGnb2; ++j)
    {
      utPositionAlloc->Add (Vector (x2->GetValue (), y2->GetValue (), ueHeight));
    }

  bsPositionAlloc->Add (Vector (0.0, 30.0, gNbHeight));
  bsPositionAlloc->Add (Vector (50.0, 0.0, gNbHeight));

  //Mobility
  mobility.SetPositionAllocator (bsPositionAlloc);
  mobility.Install (gNbNodes);

  mobility2.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Mode", StringValue ("Time"),
                              "Time", StringValue ("10s"), "Speed",
                              StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"), "Bounds",
                              StringValue ("10|40|8|22"));
  mobility2.Install (ueNodes.Get (0));

  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (39.0, 9.0, ueHeight));

  Ptr<const MobilityModel> mob = ueNodes.Get (0)->GetObject<MobilityModel> ();
  m_courseChangeFile << "0" << "\t";
  m_courseChangeFile << mob->GetObject<Node> ()->GetId () << "\t";
  m_courseChangeFile << mob->GetPosition ().x << "\t";
  m_courseChangeFile << mob->GetPosition ().y << "\t";
  m_courseChangeFile << mob->GetPosition ().z << std::endl;

  mobility.SetPositionAllocator (utPositionAlloc);
  for (uint16_t i = 1; i <= ueNumStaticGnb1 + ueNumStaticGnb2; i++)
    {
      mobility.Install (ueNodes.Get (i));
    }

  std::ofstream m_outFileUePosition;
  m_outFileUePosition.open ("UePosition.txt");
  m_outFileUePosition.setf (std::ios_base::fixed);

  if (!m_outFileUePosition.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << "UePosition.txt");
    }

  for (uint16_t i = 0; i < ueNodes.GetN (); i++)
    {
      m_outFileUePosition << ueNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x << "\t"
                          << ueNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y << "\t"
                          << ueNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z
                          << std::endl;
    }

  m_outFileUePosition.close ();

  std::cout << "gNb 1 position " << gNbNodes.Get (0)->GetObject<MobilityModel> ()->GetPosition ()
            << std::endl;
  std::cout << "gNb 2 position" << gNbNodes.Get (1)->GetObject<MobilityModel> ()->GetPosition ()
            << std::endl;
  std::cout << "UE position" << ueNodes.Get (0)->GetObject<MobilityModel> ()->GetPosition ()
            << std::endl;

  if (enableBuildings)
    {

      Ptr<Building> habCompleta;
      habCompleta = Create<Building> ();
      habCompleta->SetBoundaries (Box (20.0, 30.0, 4.0, 24.0, 0.0, 6.0));
      habCompleta->SetBuildingType (Building::Office);
      habCompleta->SetExtWallsType (Building::ConcreteWithoutWindows);
      habCompleta->SetNFloors (1);
      habCompleta->SetNRoomsX (1);
      habCompleta->SetNRoomsY (1);

      int numBloques = 5;
      int xminIzq = 20;
      int xminDer = 27;
      int ymin = 5;

      for (int i = 0; i < numBloques; i++)
        {
          Ptr<Building> bloqueIzq;
          bloqueIzq = Create<Building> ();
          bloqueIzq->SetBoundaries (Box (xminIzq, xminIzq + 3, ymin, ymin + 2, 0.0, 3.0));
          bloqueIzq->SetBuildingType (Building::Office);
          bloqueIzq->SetExtWallsType (Building::ConcreteWithoutWindows);
          bloqueIzq->SetNFloors (1);
          bloqueIzq->SetNRoomsX (1);
          bloqueIzq->SetNRoomsY (1);

          Ptr<Building> bloqueDer;
          bloqueDer = Create<Building> ();
          bloqueDer->SetBoundaries (Box (xminDer, xminDer + 3, ymin, ymin + 2, 0.0, 3.0));
          bloqueDer->SetBuildingType (Building::Office);
          bloqueDer->SetExtWallsType (Building::ConcreteWithoutWindows);
          bloqueDer->SetNFloors (1);
          bloqueDer->SetNRoomsX (1);
          bloqueDer->SetNRoomsY (1);
          ymin = ymin + 4;
        }

      BuildingsHelper::Install (gNbNodes);
      BuildingsHelper::Install (ueNodes);
      //BuildingsHelper::MakeMobilityModelConsistent (); warning-deprecated

      //MAKE consistent
      for (uint16_t i = 0; i < gNbNodes.GetN (); i++)
        {
          gNbNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
              gNbNodes.Get (i)->GetObject<MobilityModel> ());
        }

      for (uint16_t i = 0; i < ueNodes.GetN (); i++)
        {
          ueNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
              ueNodes.Get (i)->GetObject<MobilityModel> ());
        }

      TraceBuildingLoc ();
    }

  // Install nr net devices
  NetDeviceContainer gNbNetDev;
  gNbNetDev.Add (nrHelper->InstallGnbDevice (gNbNodes.Get (0), bwps1));
  gNbNetDev.Add (nrHelper->InstallGnbDevice (gNbNodes.Get (1), bwps2));

  NetDeviceContainer ueNetDevGnb1, ueNetDevGnb2;
  ueNetDevGnb1.Add (nrHelper->InstallUeDevice (ueNodes.Get (0), bwps1));
  ueNetDevGnb2.Add (nrHelper->InstallUeDevice (ueNodes.Get (0), bwps2));
  for (uint32_t i = 1; i <= ueNumStaticGnb1; i++)
    {
      ueNetDevGnb1.Add (nrHelper->InstallUeDevice (ueNodes.Get (i), bwps1));
    }

  for (uint32_t i = ueNumStaticGnb1 + 1; i <= ueNumStaticGnb1 + ueNumStaticGnb2; i++)
    {
      ueNetDevGnb2.Add (nrHelper->InstallUeDevice (ueNodes.Get (i), bwps2));
    }

  int64_t randomStream = 1;
  randomStream += nrHelper->AssignStreams (gNbNetDev, randomStream);
  randomStream += nrHelper->AssignStreams (ueNetDevGnb1, randomStream);
  randomStream += nrHelper->AssignStreams (ueNetDevGnb2, randomStream);

  nrHelper->GetGnbPhy (gNbNetDev.Get (1), 0)->SetAttribute ("TxPower", DoubleValue (txPower));

  // When all the configuration is done, explicitly call UpdateConfig ()

  for (auto it = gNbNetDev.Begin (); it != gNbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = ueNetDevGnb1.Begin (); it != ueNetDevGnb1.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = ueNetDevGnb2.Begin (); it != ueNetDevGnb2.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // connect a remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  NetDeviceContainer internetDevices2 = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  ipv4h.SetBase ("2.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces2 = ipv4h.Assign (internetDevices2);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4Address remoteHostAddr2 = internetIpIfaces2.GetAddress (1);

  std::cout << "RemoteHostAddr: " << remoteHostAddr << std::endl;

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  internet.Install (ueNodes);

  Ipv4InterfaceContainer ueIpIfaceGnb1 =
      epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDevGnb1));
  Ipv4InterfaceContainer ueIpIfaceGnb2 =
      epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDevGnb2));

  //Ver que IP tiene cada ueNas
  std::cout << "IP Interface 1: " << ueNodes.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0)
            << std::endl;
  std::cout << "IP Interface 2: " << ueNodes.Get (0)->GetObject<Ipv4> ()->GetAddress (2, 0)
            << std::endl;

  std::cout << "NetDevice address 1: " << ueNetDevGnb1.Get (0)->GetAddress () << std::endl;
  std::cout << "NetDevice address 2: " << ueNetDevGnb2.Get (0)->GetAddress () << std::endl;

  // Set the default gateway for the UEs
  for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (j)->GetObject<Ipv4> ());
      if (j == 0)
        {
          ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("1.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                              epcHelper->GetUeDefaultGatewayAddress (), 1);
          ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("1.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                              epcHelper->GetUeDefaultGatewayAddress (), 2);
          ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("2.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                              epcHelper->GetUeDefaultGatewayAddress (), 1);
        }
      else
        {
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }
    }

  //////////////////////////////////////////////////////////
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  uint16_t ulPort = 2000;
  uint16_t dlPort = 3000;
  uint16_t n1 = 1;
  uint16_t n2 = 1;

  for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
    {
      ++ulPort;
      ++dlPort;

      Ptr<EpcTft> tftUl = Create<EpcTft> ();
      EpcTft::PacketFilter ulpf;
      ulpf.localPortStart = ulPort;
      ulpf.localPortEnd = ulPort;
      tftUl->Add (ulpf);

      enum EpsBearer::Qci qUl;
      qUl = EpsBearer::NGBR_LOW_LAT_EMBB;
      EpsBearer bearerUl (qUl);

      if (j != 0)
        { // DL traffic
          PacketSinkHelper sinkDL ("ns3::UdpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), dlPort));

          if (j <= ueNumStaticGnb1)
            {
              //DL
              OnOffHelper sourceDLNR ("ns3::UdpSocketFactory",
                                      InetSocketAddress (ueIpIfaceGnb1.GetAddress (n1), dlPort));
              // Set the amount of data to send in bytes.  Zero is unlimited.
              sourceDLNR.SetAttribute ("DataRate", DataRateValue (DataRate ("1Mb/s")));
              sourceDLNR.SetAttribute ("OffTime",
                                       StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
              clientApps.Add (sourceDLNR.Install (remoteHost));
              n1++;
            }
          else
            {
              //DL
              OnOffHelper sourceDLNR ("ns3::UdpSocketFactory",
                                      InetSocketAddress (ueIpIfaceGnb2.GetAddress (n2), dlPort));
              // Set the amount of data to send in bytes.  Zero is unlimited.
              sourceDLNR.SetAttribute ("DataRate", DataRateValue (DataRate ("1Mb/s")));
              sourceDLNR.SetAttribute ("OffTime",
                                       StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
              clientApps.Add (sourceDLNR.Install (remoteHost));
              n2++;
            }

          //DL
          serverApps.Add (sinkDL.Install (ueNodes.Get (j)));
        }
      else
        { // UE 0
          UdpServerHelper dlPacketSinkHelper (dlPort);
          serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (0))); // DL
          UdpClientHelper dlClient (ueIpIfaceGnb1.GetAddress (0), dlPort); // DL

          std::cout << "ADDRESS UE0- NETDEVICE0 " << ueIpIfaceGnb1.GetAddress (0) << std::endl;
          std::cout << "ADDRESS UE0- NETDEVICE1 " << ueIpIfaceGnb2.GetAddress (0) << std::endl;

          dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (udpInterval)));
          dlClient.SetAttribute ("PacketSize", UintegerValue (udpPacketSize));
          dlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));

          clientApps.Add (dlClient.Install (remoteHost)); // DL
          std::cout << remoteHostAddr2 << std::endl;
        }
    }

  serverApps.Start (MilliSeconds (100));
  clientApps.Start (Seconds (udpAppStartTime));
  serverApps.Stop (Seconds (simTime));
  clientApps.Stop (Seconds (simTime - 1));

  // attach UEs to the closest eNB
  //nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);
  std::cout << "Referencia NetDevice1: " << ueNetDevGnb1.Get (0) << std::endl;
  std::cout << "Referencia NetDevice2: " << ueNetDevGnb2.Get (0) << std::endl;

  for (uint32_t i = 0; i < ueNetDevGnb1.GetN (); i++)
    {
      nrHelper->AttachToEnb (ueNetDevGnb1.Get (i), gNbNetDev.Get (0));
    }

  for (uint32_t i = 0; i < ueNetDevGnb2.GetN (); i++)
    {
      nrHelper->AttachToEnb (ueNetDevGnb2.Get (i), gNbNetDev.Get (1));
    }

  // Add X2 interface
  epcHelper->AddX2Interface (gNbNodes.Get (0), gNbNodes.Get (1));

  Ptr<LteEnbRrc> masterGnbRrc = gNbNetDev.Get (0)->GetObject<NrGnbNetDevice> ()->GetRrc ();
  Ptr<LteEnbRrc> secondaryGnbRrc = gNbNetDev.Get (1)->GetObject<NrGnbNetDevice> ()->GetRrc ();
  masterGnbRrc->SetMasterGnb (true);
  masterGnbRrc->SetSecondaryGnb (false);
  secondaryGnbRrc->SetMasterGnb (false);
  secondaryGnbRrc->SetSecondaryGnb (true);
  uint16_t sourceCellId = gNbNetDev.Get (0)->GetObject<NrGnbNetDevice> ()->GetCellId () + 1;
  uint16_t targetCellId = gNbNetDev.Get (1)->GetObject<NrGnbNetDevice> ()->GetCellId () + 1;

  Simulator::Schedule (MilliSeconds (500), &ActivatePacketDuplication, masterGnbRrc,
                       ueNetDevGnb1.Get (0), sourceCellId, targetCellId, ueNetDevGnb2.Get (0));

  // enable the traces provided by the nr module
  nrHelper->EnableTracesForANodeDevice ("*", "*");

  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));

  FlowMonitorHelper flowmonHelper;
  NodeContainer endpointNodes;
  endpointNodes.Add (remoteHost);
  endpointNodes.Add (ueNodes);

  Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install (endpointNodes);
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));

  std::cout << "######### Simulator::Run() ###########" << std::endl;
  std::cout << std::endl;

  //RADIO ENVIRONMENT MAP
  if (useREM)
    {
      uint16_t remBwpId = 0;
      //Radio Environment Map Generation for ccId 0
      Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper> ();
      //Rem parameters
      double xMin = -60.0;
      double xMax = 60.0;
      uint16_t xRes = 100;
      double yMin = -60.0;
      double yMax = 60.0;
      uint16_t yRes = 100;
      double z = 1.5;
      std::string simTagREM = "test";

      remHelper->SetMinX (xMin);
      remHelper->SetMaxX (xMax);
      remHelper->SetResX (xRes);
      remHelper->SetMinY (yMin);
      remHelper->SetMaxY (yMax);
      remHelper->SetResY (yRes);
      remHelper->SetZ (z);
      remHelper->SetSimTag (simTagREM);
      remHelper->SetRemMode (NrRadioEnvironmentMapHelper::COVERAGE_AREA);

      gNbNetDev.Get (0)
          ->GetObject<NrGnbNetDevice> ()
          ->GetPhy (remBwpId)
          ->GetBeamManager ()
          ->ChangeToQuasiOmniBeamformingVector ();
      gNbNetDev.Get (1)
          ->GetObject<NrGnbNetDevice> ()
          ->GetPhy (remBwpId)
          ->GetBeamManager ()
          ->ChangeToQuasiOmniBeamformingVector ();

      std::string typeOfRem = "DlRem";
      if (typeOfRem.compare ("DlRem") == 0)
        {
          Ptr<NetDevice> ueRemDevice = ueNetDevGnb2.Get (0);
          remHelper->CreateRem (gNbNetDev.Get (1), ueRemDevice, remBwpId);
        }
      else if (typeOfRem.compare ("UlRem") == 0)
        {
          Ptr<NetDevice> gnbRemDevice = gNbNetDev.Get (0);
          remHelper->CreateRem (ueNetDevGnb1, gnbRemDevice, remBwpId);
        }
      else
        {
          NS_ABORT_MSG ("typeOfRem not supported. "
                        "Choose among 'DlRem', 'UlRem'.");
        }
    }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  m_algoritmEvent.Cancel ();
  //SaveTimePositions ("timePositions.txt",g_timeposition);
  m_courseChangeFile.close ();

  m_outSinrFile.close ();
  m_outSnrFile.close ();
  m_outRssiFile.close ();

  m_outSinrFileGnb.close ();
  m_outSnrFileGnb.close ();
  m_outRssiFileGnb.close ();

  m_outSinrFileGnb1.close ();
  m_outSnrFileGnb1.close ();
  m_outRssiFileGnb1.close ();

  // Print per-flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

  double averageFlowThroughput = 0.0;
  double averageFlowDelay = 0.0;

  std::ofstream outFile;
  std::string filename = outputDir + "/" + simTag;
  outFile.open (filename.c_str (), std::ofstream::out | std::ofstream::app);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return 1;
    }
  outFile.setf (std::ios_base::fixed);

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin ();
       i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      std::stringstream protoStream;
      protoStream << (uint16_t) t.protocol;
      if (t.protocol == 6)
        {
          protoStream.str ("TCP");
        }
      if (t.protocol == 17)
        {
          protoStream.str ("UDP");
        }
      outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
              << t.destinationAddress << ":" << t.destinationPort << ") proto "
              << protoStream.str () << "\n";
      outFile << "  Tx Packets: " << i->second.txPackets << "\n";
      outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
      outFile << "  TxOffered:  "
              << i->second.txBytes * 8.0 / (simTime - udpAppStartTime) / 1000 / 1000 << " Mbps\n";
      outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
      if (i->second.rxPackets > 0)
        {
          // Measure the duration of the flow from receiver's perspective
          double rxDuration =
              i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ();

          averageFlowThroughput += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
          averageFlowDelay += 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;

          outFile << "  Throughput: " << i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000
                  << " Mbps\n";
          outFile << "  Mean delay:  "
                  << 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets << " ms\n";
          //outFile << "  Mean upt:  " << i->second.uptSum / i->second.rxPackets / 1000/1000 << " Mbps \n";
          outFile << "  Mean jitter:  "
                  << 1000 * i->second.jitterSum.GetSeconds () / i->second.rxPackets << " ms\n";
        }
      else
        {
          outFile << "  Throughput:  0 Mbps\n";
          outFile << "  Mean delay:  0 ms\n";
          outFile << "  Mean upt:  0  Mbps \n";
          outFile << "  Mean jitter: 0 ms\n";
        }
      outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

  outFile << "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size () << "\n";
  outFile << "  Mean flow delay: " << averageFlowDelay / stats.size () << "\n";
  outFile.close ();

  Simulator::Destroy ();
  return 0;
}
