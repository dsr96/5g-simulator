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

/**
  *
  * To run the simulation with the default configuration one shall run the
  * following in the command line:
  *
  * ./waf --run scratch/logistics-scenario
  *
  */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("logistics-scenario");

Ptr<Node> remoteHost;

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

std::unordered_map<uint32_t, std::pair<Vector, uint8_t>> umap;
std::unordered_map<uint32_t, bool> umapUpdate;
//map that contains for each nodeID the associated remotehost udp client app
std::unordered_map<uint32_t, uint8_t> umapAppRemoteHost;

double
DetermineEMBBDistanceY (int pointY, double wpY)
{
  double distY = 0.0;
  switch (pointY) // Points Y: 1 (5 m), 2 (9 m), 3 (13 m), 4 (17 m), 5 (21 m) y 6 (25 m)
    {
    case 1:
      distY = wpY - 5;
      break;
    case 2:
      distY = wpY - 9;
      break;
    case 3:
      distY = wpY - 13;
      break;
    case 4:
      distY = wpY - 17;
      break;
    case 5:
      distY = wpY - 21;
      break;
    case 6:
      distY = wpY - 25;
      break;
    }
  return distY;
}

double
DetermineEMBBDistanceX (int pointX, double wpX)
{
  double distX = 0.0;
  switch (pointX) // Points 4: 1 (35 m), 2 (30 m), 3 (25 m)
    {
    case 1:
      distX = wpX - 35;
      break;
    case 2:
      distX = wpX - 30;
      break;
    case 3:
      distX = wpX - 25;
      break;
    }
  return distX;
}

void
UpdateEMBBWaypoints (double timeGranularity, double wpTime, double vel, double xGranurality,
                     double yGranurality, double wpX, double wpY, double ueHeight,
                     Ptr<WaypointMobilityModel> mob)
{
  Ptr<UniformRandomVariable> yEMBB = CreateObject<UniformRandomVariable> ();
  yEMBB->SetAttribute ("Min", DoubleValue (1));
  yEMBB->SetAttribute ("Max", DoubleValue (6));

  int pointY = yEMBB->GetInteger ();
  //std::cout << "Hall selected: " << pointY << std::endl;
  double distY = DetermineEMBBDistanceY (pointY, wpY);

  int numwpY = abs (distY) / vel / timeGranularity;

  for (int y1 = 0; y1 < numwpY; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0) //direction
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }

  Ptr<UniformRandomVariable> xEMBB = CreateObject<UniformRandomVariable> ();
  xEMBB->SetAttribute ("Min", DoubleValue (1));
  xEMBB->SetAttribute ("Max", DoubleValue (3));

  int pointX = xEMBB->GetInteger ();
  //std::cout << "Point 4 selected: " << pointX << std::endl;
  int distX = DetermineEMBBDistanceX (pointX, wpX);

  int numwpX = abs (distX) / vel / timeGranularity;

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX - xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX + xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int y1 = 0; y1 < numwpY + 1; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0) // direction
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }
}

double
DetermineURLLCT1DistanceY (double wpY)
{
  double distY = wpY - 25;
  return distY;
}

double
DetermineURLLCT1DistanceX (double wpX)
{
  double distX = wpX - (-6);
  return distX;
}

void
UpdateURLLCT1Waypoints (double timeGranularity, double wpTime, double vel, double xGranurality,
                        double yGranurality, double wpX, double wpY, double ueHeight,
                        Ptr<WaypointMobilityModel> mob)
{
  double distY = DetermineURLLCT1DistanceY (wpY);

  int numwpY = abs (distY) / vel / timeGranularity;

  for (int y1 = 0; y1 < numwpY; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0)
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }

  int distX = DetermineURLLCT1DistanceX (wpX);

  int numwpX = abs (distX) / vel / timeGranularity;

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX - xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX + xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int y1 = 0; y1 < numwpY + 1; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0) // direction
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }
}

double
DetermineURLLCT2DistanceY (int pointY, double wpY)
{
  double distY = 0.0;
  switch (pointY) // Points Y: 1 (5 m), 2 (9 m), 3 (13 m), 4 (17 m), 5 (21 m) y 6 (25 m)
    {
    case 1:
      distY = wpY - 5;
      break;
    case 2:
      distY = wpY - 9;
      break;
    case 3:
      distY = wpY - 13;
      break;
    case 4:
      distY = wpY - 17;
      break;
    case 5:
      distY = wpY - 21;
      break;
    case 6:
      distY = wpY - 25;
      break;
    }
  return distY;
}

double
DetermineURLLCT2DistanceX (int pointX, double wpX)
{
  double distX = 0.0;
  switch (pointX) // Points 4: 1 (35 m), 2 (30 m), 3 (25 m) + 1 para que llegue
    {
    case 1:
      distX = wpX - 26;
      break;
    case 2:
      distX = wpX - 31;
      break;
    case 3:
      distX = wpX - 36;
      break;
    }
  return distX;
}

void
UpdateURLLCT2Waypoints (double timeGranularity, double wpTime, double vel, double xGranurality,
                        double yGranurality, double wpX, double wpY, double ueHeight,
                        Ptr<WaypointMobilityModel> mob)
{
  Ptr<UniformRandomVariable> yURLLCT2 = CreateObject<UniformRandomVariable> ();
  yURLLCT2->SetAttribute ("Min", DoubleValue (1));
  yURLLCT2->SetAttribute ("Max", DoubleValue (6));

  int pointY = yURLLCT2->GetInteger ();
  //std::cout << "Hall selected: " << pointY << std::endl;
  double distY = DetermineURLLCT2DistanceY (pointY, wpY);

  int numwpY = abs (distY) / vel / timeGranularity;

  for (int y1 = 0; y1 < numwpY; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0) // direction
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }

  Ptr<UniformRandomVariable> xURLLCT2 = CreateObject<UniformRandomVariable> ();
  xURLLCT2->SetAttribute ("Min", DoubleValue (1));
  xURLLCT2->SetAttribute ("Max", DoubleValue (3));

  int pointX = xURLLCT2->GetInteger ();
  //std::cout << "Point 4 selected: " << pointX << std::endl;
  int distX = DetermineURLLCT2DistanceX (pointX, wpX);

  int numwpX = abs (distX) / vel / timeGranularity;

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX + xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int x1 = 0; x1 < numwpX; x1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      wpX = wpX - xGranurality;
      wpTime = wpTime + timeGranularity;
    }

  for (int y1 = 0; y1 < numwpY + 1; y1++)
    {
      mob->AddWaypoint (Waypoint (Seconds (wpTime), Vector (wpX, wpY, ueHeight)));
      if (distY > 0) // direction
        {
          wpY = wpY + yGranurality;
          wpTime = wpTime + timeGranularity;
        }
      else
        {
          wpY = wpY - yGranurality;
          wpTime = wpTime + timeGranularity;
        }
    }
}

bool
double_equals (double a, double b, double epsilon = 0.001)
{
  return std::abs (a - b) < epsilon;
}

static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  uint32_t nodeId = mobility->GetObject<Node> ()->GetId ();
  double posX = mobility->GetPosition ().x;
  double posY = mobility->GetPosition ().y;
  double posZ = mobility->GetPosition ().z;

  /*std::cout << "Course change: " << std::endl;
 	 std::cout << "Time: " << Simulator::Now().GetSeconds() << std::endl;
 	 std::cout << "NodeId: " << nodeId << std::endl;
 	 std::cout  << "Position: " << posX << " " << posY  << " " << posZ << std::endl;
 	 std::cout << std::endl; */

  m_courseChangeFile << Simulator::Now ().GetSeconds () << "\t";
  m_courseChangeFile << nodeId << "\t";
  m_courseChangeFile << posX << "\t";
  m_courseChangeFile << posY << "\t";
  m_courseChangeFile << posZ << std::endl;

  Vector initialPosition = umap[nodeId].first;
  uint8_t type = umap[nodeId].second; // 1-EMBB, 2-URLLCT1, 3-URLLCT2

  //Comparison between double values must use double_equals function
  // two values that should be equal may not be due to arithmetic rounding errors
  if ((double_equals (initialPosition.x, posX) == true) &&
      (double_equals (initialPosition.y, posY) == true))
    {
      bool updating = umapUpdate[nodeId];
      if (updating)
        {
          umapUpdate[nodeId] = false;
        }
      else
        {
          switch (type)
            {
              case 1: //EMBB
              {
                Ptr<UniformRandomVariable> periodicity = CreateObject<UniformRandomVariable> ();
                periodicity->SetAttribute ("Min", DoubleValue (4)); // Minutes
                periodicity->SetAttribute ("Max", DoubleValue (6));

                double timeGranularity = 0.1;
                double wpTime = Simulator::Now ().GetSeconds () + periodicity->GetValue () * 60;
                double vel = 1; // m/s
                double xGranurality = vel * timeGranularity;
                double yGranurality = vel * timeGranularity;
                double wpX = initialPosition.x; // Initial X value
                double wpY = initialPosition.y; // Initial Y value
                double ueHeight = 1.5;
                Ptr<WaypointMobilityModel> mob =
                    mobility->GetObject<Node> ()->GetObject<WaypointMobilityModel> ();
                mob->EndMobility ();

                UpdateEMBBWaypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX,
                                     wpY, ueHeight, mob);
                break;
              }
              case 2: //URLLC Type 1
              {

                double timeGranularity = 0.1;
                double wpTime = Simulator::Now ().GetSeconds ();
                double vel = 1; // m/s
                double xGranurality = vel * timeGranularity;
                double yGranurality = vel * timeGranularity;
                double wpX = initialPosition.x; // Initial X value
                double wpY = initialPosition.y; // Initial Y value
                double ueHeight = 1.5;
                Ptr<WaypointMobilityModel> mob =
                    mobility->GetObject<Node> ()->GetObject<WaypointMobilityModel> ();
                mob->EndMobility ();

                // Resume Tx packets when AGV starts to move again
                //udpC->ResumeTx (wpTime - Simulator::Now().GetSeconds());

                UpdateURLLCT1Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality,
                                        wpX, wpY, ueHeight, mob);
                break;
              }
              case 3: //URLLC Type 2
              {

                double timeGranularity = 0.1;
                double wpTime = Simulator::Now ().GetSeconds ();

                double vel = 1; // m/s
                double xGranurality = vel * timeGranularity;
                double yGranurality = vel * timeGranularity;
                double wpX = initialPosition.x; // Initial X value.
                double wpY = initialPosition.y; // Initial Y value
                double ueHeight = 1.5;
                Ptr<WaypointMobilityModel> mob =
                    mobility->GetObject<Node> ()->GetObject<WaypointMobilityModel> ();
                mob->EndMobility ();

                // Resume Tx packets when AGV starts to move again
                //udpC->ResumeTx (wpTime - Simulator::Now().GetSeconds());

                UpdateURLLCT2Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality,
                                        wpX, wpY, ueHeight, mob);
                break;
              }
            }

          umapUpdate[nodeId] = true;
        }
    }
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

EventId m_timeEvent;
void
PrintSimulationTime (double simTime)
{
  std::cout << "Current SIMULATION TIME (s) " << Simulator::Now ().GetSeconds () << std::endl;
  std::cout << "Simulation progress: " << Simulator::Now ().GetSeconds () / simTime * 100 << " %"
            << std::endl
            << std::endl;
  m_timeEvent = Simulator::Schedule (Seconds (1), &PrintSimulationTime, simTime);
}

void
PrintIMSI_RNTI (NetDeviceContainer mmtcUeNetDev, NetDeviceContainer embbUeNetDev,
                NetDeviceContainer urllcUeNetDev)
{
  //Create a file with IMSI-RNTI associate to type of Service (MMTC, EMBB, URLLC)
  std::ofstream m_outFileIMSI_RNTI;
  m_outFileIMSI_RNTI.open ("IMSI-RNTI-Table.txt");
  m_outFileIMSI_RNTI.setf (std::ios_base::fixed);

  if (!m_outFileIMSI_RNTI.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << "IMSI-RNTI-Table.txt");
    }

  m_outFileIMSI_RNTI << "IMSI" << "\t" << "RNTI" << "\t" << "Service" << std::endl;

  for (uint32_t j = 0; j < mmtcUeNetDev.GetN (); ++j)
    {
      m_outFileIMSI_RNTI << mmtcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetImsi ()
                         << "\t"
                         << mmtcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ()
                         << "\t" << "MMTC" << std::endl;
    }

  for (uint32_t j = 0; j < embbUeNetDev.GetN (); ++j)
    {
      m_outFileIMSI_RNTI << embbUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetImsi ()
                         << "\t"
                         << embbUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti ()
                         << "\t" << "EMBB" << std::endl;
    }

  for (uint32_t j = 0; j < urllcUeNetDev.GetN (); ++j)
    {
      if (j == 0)
        {
          m_outFileIMSI_RNTI
              << urllcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetImsi () << "\t"
              << urllcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti () << "\t"
              << "URLLCT1" << std::endl;
        }
      else
        {
          m_outFileIMSI_RNTI
              << urllcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetImsi () << "\t"
              << urllcUeNetDev.Get (j)->GetObject<NrUeNetDevice> ()->GetRrc ()->GetRnti () << "\t"
              << "URLLC" << std::endl;
        }
    }
  m_outFileIMSI_RNTI.close ();
}

int
main (int argc, char *argv[])
{

  // enable logging or not
  bool logging = false;
  if (logging)
    {
      LogComponentEnable ("UdpClient", LOG_LEVEL_ALL);
    }

  LogComponentEnable ("CcBwpHelper", LOG_LEVEL_ALL);

  // set simulation time and appStart
  double simTime = 600;
  double udpAppStartTime = 0.2; //seconds

  //other simulation parameters default values
  uint16_t gNbNum = 1;
  uint16_t urllcUEsType1 = 1;
  uint16_t urllcUEsType2 = 9;
  uint16_t embbUEs = 5;
  uint16_t mmtcUEs = 28; // 1000 devices

  // Frequency band and transmission power (gNB and UE)
  double centralFrequency = 3.7e9;
  double centralFrequencyMMTC = 0.5e9; // 500 MHz frequency for MMTC devices
  double totalBandwidth = 20e6;
  double bandwidthEMBB = 11e6;
  double bandwidthURLLC = 6e6;
  double bandwidthMMTC = 3e6;
  double txPower = 15; //dBm
  double txPowerUE = 10; //dBm

  // Numerology
  uint16_t numerology = 2;
  uint16_t symbolsPerSlot = 14;

  // URLLC Traffic profile
  uint32_t urllcPacketSize = 64; // bytes
  double urllcInterval = 100; //ms (10 messages/second)

  // MMTC Traffic profile
  uint32_t mmtcPacketSize = 64; // bytes
  double mmtcInterval = 15 * 60; // minutes
  uint8_t mmtcRepetitions = 4;

  // Modulation scheme (adaptive)
  uint8_t fixedMcsDL = 20;
  uint8_t fixedMcsUL = 5;
  bool useFixedMcs = false; // false-> adaptive, true->fixed

  // scenario (A or B)
  // A-> workers moving, smart tags working, no AGVs.
  // B-> no workers, smart tags working, AGVs moving.
  // C-> all, workers, tags and AGVs
  std::string scenarioAB = "C";

  // Where we will store the output files.
  std::string simTag = "statistics.txt";
  std::string outputDir = "./";

  bool shannonModel = false;
  bool enableBuildings = true;

  // 3GPP 38.901 Industrial scenario Dense High parameters
  std::string industrialscenario = "InF-DH";
  double lowClutterDensity = 0.2;
  double highClutterDensity = 0.6;
  double InFVolume = 12000; //40x30x10 m3;
  double InFTotalSurface = 1200; //40x30 m2;
  double clutterHeight = 6; //metros
  bool enableShadowing = true;

  // Harq (default values)
  bool harqEnabled = true;
  uint8_t harqProcesses = 20;

  bool BuildingsEnabled = true;

  int64_t randomStream = 1;

  //Beamforming
  std::string m_beamforming = "REALISTIC";

  CommandLine cmd;
  cmd.AddValue ("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
  cmd.AddValue ("numerology", "The numerology to be used for URLLC.", numerology);
  cmd.AddValue ("txPower", "Tx power to be configured to gNB", txPower);
  cmd.AddValue ("simTag",
                "tag to be appended to output filenames to distinguish simulation campaigns",
                simTag);
  cmd.AddValue ("outputDir", "directory where to store simulation results", outputDir);
  cmd.AddValue ("frequency", "The system frequency", centralFrequency);
  cmd.AddValue ("frequencyMMTC", "The system frequency for MMTC devices", centralFrequencyMMTC);
  cmd.AddValue ("totalBandwidth", "The system bandwidth", totalBandwidth);
  cmd.AddValue ("bandwidthEMBB", "The system bandwidth for EMBB", bandwidthEMBB);
  cmd.AddValue ("bandwidthURLLC", "The system bandwidth for URLLC", bandwidthURLLC);
  cmd.AddValue ("bandwidthMMTC", "The system bandwidth for MMTC", bandwidthMMTC);
  cmd.AddValue ("fixedMcsDL",
                "The fixed MCS DL that will be used in this example if useFixedMcs is configured "
                "to true (1).",
                fixedMcsDL);
  cmd.AddValue ("fixedMcsUL",
                "The fixed MCS UL that will be used in this example if useFixedMcs is configured "
                "to true (1).",
                fixedMcsUL);
  cmd.AddValue ("useFixedMcs", "Whether to use fixed mcs, normally used for testing purposes",
                useFixedMcs);
  cmd.AddValue ("txPowerUE", "Potencia de tx de UE NR en dBm", txPowerUE);
  cmd.AddValue ("symbolsPerSlot", "Número de símbolos por slot", symbolsPerSlot);
  cmd.AddValue ("industrialscenario", "Selecciona InF-SH o InF-DH", industrialscenario);
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
  cmd.AddValue ("enableBuildings", "Insertar edificios al escenario", enableBuildings);
  cmd.AddValue ("buildingsEnabled", "Activate/Deactivate buildings propagation loss model",
                BuildingsEnabled);
  cmd.AddValue ("enableShadowing", "Activate shadowing in propagation loss model", enableShadowing);
  cmd.AddValue ("beamformingType", "IDEAL or REALISTIC", m_beamforming);
  cmd.AddValue ("harqEnabled", "Activate HARQ", harqEnabled);
  cmd.AddValue ("harqProcesses", "Number of Harq processes", harqProcesses);
  cmd.AddValue ("scenarioAB", "Scenario A (MMTC and EMMB) or B (MMTC and URLLC)", scenarioAB);
  cmd.AddValue ("simTime", "Simulation Time", simTime);
  cmd.AddValue ("numMMTC", "NUM mmtc devices", mmtcUEs);
  cmd.AddValue ("numEMBB", "NUM embb devices", embbUEs);
  cmd.AddValue ("numURLLCType1", "NUM urllc devices type 1", urllcUEsType1);
  cmd.AddValue ("numURLLCType2", "NUM urllc devices type 2", urllcUEsType2);
  cmd.AddValue ("randomStream", "NR Random Stream", randomStream);
  cmd.AddValue ("mmtcRepetitions", "Number of packet repetitions at application for MMTC-Traffic",
                mmtcRepetitions);

  cmd.Parse (argc, argv);

  NS_ABORT_IF (bandwidthMMTC + bandwidthEMBB + bandwidthURLLC > totalBandwidth);

  m_courseChangeFile.open ("timePositions.txt");
  if (!m_courseChangeFile.is_open ())
    {
      NS_ABORT_MSG ("Can't open file " << "timePositions.txt");
    }

  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::EpsBearer::Release", UintegerValue (15));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                      TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::ThreeGppChannelModel::InFVolume", DoubleValue (InFVolume));
  Config::SetDefault ("ns3::ThreeGppChannelModel::InFTotalSurface", DoubleValue (InFTotalSurface));
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::BuildingsEnabled",
                      BooleanValue (BuildingsEnabled));
  Config::SetDefault ("ns3::NrHelper::HarqEnabled", BooleanValue (harqEnabled));

  Config::SetDefault ("ns3::NrGnbPhy::N2Delay", UintegerValue (0));
  Config::SetDefault ("ns3::NrGnbPhy::N1Delay", UintegerValue (0));

  //Noise figure (38901 v16.1.0)
  Config::SetDefault ("ns3::NrUePhy::NoiseFigure", DoubleValue (9.0));

  if (industrialscenario == "InF-DH")
    {
      Config::SetDefault ("ns3::ThreeGppIndoorFactoryDHChannelConditionModel::HighClutterDensity",
                          DoubleValue (highClutterDensity));
      Config::SetDefault ("ns3::ThreeGppIndoorFactoryDHChannelConditionModel::ClutterHeight",
                          DoubleValue (clutterHeight));
    }
  else if (industrialscenario == "InF-SH") // InF-SH
    {
      Config::SetDefault ("ns3::ThreeGppIndoorFactorySHChannelConditionModel::LowClutterDensity",
                          DoubleValue (lowClutterDensity));
      Config::SetDefault ("ns3::ThreeGppIndoorFactorySHChannelConditionModel::ClutterHeight",
                          DoubleValue (clutterHeight));
    }

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  nrHelper->SetUeMacAttribute ("NumHarqProcess", UintegerValue (harqProcesses));
  nrHelper->SetGnbMacAttribute ("NumHarqProcess", UintegerValue (harqProcesses));

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  // Core latency
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

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

  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1; // in this example, both bands have a single CC
  BandwidthPartInfo::Scenario scenario;

  if (industrialscenario == "InF-DH")
    {
      scenario = BandwidthPartInfo::InF_DH_nLoS;
    }
  else if (industrialscenario == "InF-SH")
    {
      scenario = BandwidthPartInfo::InF_SH_nLoS;
    }
  else
    {
      scenario = BandwidthPartInfo::InH_OfficeMixed_nLoS;
    }

  CcBwpCreator::SimpleOperationBandConf bandConf1 (centralFrequencyMMTC, bandwidthMMTC,
                                                   numCcPerBand, scenario);
  CcBwpCreator::SimpleOperationBandConf bandConf2 (centralFrequency, bandwidthEMBB + bandwidthURLLC,
                                                   numCcPerBand, scenario);

  bandConf2.m_numBwp = 2; // Here, bandFdd will have 2 BWPs

  // Bandwidth for each bwp in the CC (slices)
  std::unordered_map<uint8_t, double> bwpBandwidthSlices;
  bwpBandwidthSlices.insert (std::make_pair (0, bandwidthEMBB));
  bwpBandwidthSlices.insert (std::make_pair (1, bandwidthURLLC));
  bandConf2.m_bwpBandwidthSlices = bwpBandwidthSlices;

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf1);
  OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCcMod (bandConf2);

  std::cout << "MMTC BW: " << band1.m_cc[0]->m_bwp[0]->m_channelBandwidth << std::endl;
  std::cout << "EMBB BW0: " << band2.m_cc[0]->m_bwp[0]->m_channelBandwidth << std::endl;
  std::cout << "URLLC BW1: " << band2.m_cc[0]->m_bwp[1]->m_channelBandwidth << std::endl;

  /*
   * The configured spectrum division is:
   * |------------Band1--------------|--------------Band2---------------|
   * |------------CC0------------------|--------------CC1-------------------|
   * |------------BWP0-----------------|------BWP1-------|-------BWP2-------|
   *
   * Band1: mMTC; BWP0: mMTC
	 * Band2: EMBB, URLLC -- BWP1: eMBB, BWP2: URLLC
   */

  nrHelper->InitializeOperationBand (&band1); //MMTC
  nrHelper->InitializeOperationBand (&band2); //EMBB URLLC
  allBwps = CcBwpCreator::GetAllBwps ({band1, band2});

  /*
  * BWP parameters
	*/
  uint32_t bwpIdForMMTC = 0;
  uint32_t bwpIdForEMBB = 1;
  uint32_t bwpIdForURLLC = 2;

  // Type of GBR does not affect at the scheduler, only it is used to identify the type of traffic and BWP
  // to deliver the traffic to the specific BWP/Slice
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("GBR_GAMING", UintegerValue (bwpIdForMMTC));
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (bwpIdForEMBB));
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("GBR_CONV_VIDEO", UintegerValue (bwpIdForURLLC));

  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_GAMING", UintegerValue (bwpIdForMMTC));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (bwpIdForEMBB));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_CONV_VIDEO", UintegerValue (bwpIdForURLLC));

  /*
   * Continue setting the parameters which are common to all the nodes, like the
   * gNB transmit power or numerology.
   */
  //nrHelper->SetGnbPhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetGnbPhyAttribute ("SymbolsPerSlot", UintegerValue (symbolsPerSlot));
  //nrHelper->SetGnbPhyAttribute ("Numerology", UintegerValue (numerology));
  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPowerUE));

  // Scheduler
  nrHelper->SetSchedulerTypeId (TypeId::LookupByName ("ns3::NrMacSchedulerTdmaRR"));
  nrHelper->SetSchedulerAttribute ("FixedMcsDl", BooleanValue (useFixedMcs));
  nrHelper->SetSchedulerAttribute ("FixedMcsUl", BooleanValue (useFixedMcs));
  nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (fixedMcsDL));
  nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (fixedMcsUL));

  if (useFixedMcs == true)
    {
      nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (fixedMcsDL));
      nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (fixedMcsUL));
    }

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999)); //10 MB

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

  Config::SetDefault ("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue (true));

  nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (enableShadowing));

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

  /*
   *  Create the gNB and UE nodes according to the network topology
   */
  NodeContainer gNbNodes;
  NodeContainer urllcNodes, mmtcNodes, embbNodes;
  mmtcNodes.Create (mmtcUEs);

  if (scenarioAB == "A") //no AGVs
    {
      embbNodes.Create (embbUEs);
    }
  else if (scenarioAB == "B") //no workers
    {
      urllcNodes.Create (urllcUEsType1 + urllcUEsType2);
    }
  else // scenario C, workers and AGVs
    {
      embbNodes.Create (embbUEs);
      urllcNodes.Create (urllcUEsType1 + urllcUEsType2);
    }

  gNbNodes.Create (gNbNum);

  /* --------------------------------------------- */
  // UE and gNB Mobility
  MobilityHelper gnbMobility;
  MobilityHelper mmtcMobility, embbMobility, urllcMobility;
  Ptr<ListPositionAllocator> bsPositionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> mmtcPositionAlloc = CreateObject<ListPositionAllocator> ();

  const double gNbHeight = 10;
  const double ueHeight = 1.5;

  // MMTC mobility (smart tags) -> Fixed in a random position in the racks
  // 7 racks, for each rack mmtcUEs/7 -> mmtcUEs = 245, 35 UEs in a racks
  Ptr<UniformRandomVariable> xMMTC = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> zMMTC = CreateObject<UniformRandomVariable> ();
  xMMTC->SetAttribute ("Min", DoubleValue (25));
  xMMTC->SetAttribute ("Max", DoubleValue (35));
  zMMTC->SetAttribute ("Min", DoubleValue (0.1));
  zMMTC->SetAttribute ("Max", DoubleValue (5.9)); //Storage racks have 6 meters of height

  Ptr<UniformRandomVariable> yMMTC =
      CreateObject<UniformRandomVariable> (); // Modify SetAttribute in for loop
  double yMMTCmin = 2;
  double yMMTCmax = 4;
  yMMTC->SetAttribute ("Min", DoubleValue (yMMTCmin));
  yMMTC->SetAttribute ("Max", DoubleValue (yMMTCmax));

  mmtcMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  for (uint32_t j = 0; j < mmtcNodes.GetN (); j++)
    {
      if (j == 4 || j == 8 || j == 12 || j == 16 || j == 20 ||
          j == 24) // 4 per rack, with 28 MMTC devices
        {
          //Increase yMin and yMaxValue (from bottom to top rack)
          yMMTCmin = yMMTCmin + 4;
          yMMTCmax = yMMTCmax + 4;
          yMMTC->SetAttribute ("Min", DoubleValue (yMMTCmin));
          yMMTC->SetAttribute ("Max", DoubleValue (yMMTCmax));
        }

      mmtcMobility.Install (mmtcNodes.Get (j));
      mmtcNodes.Get (j)->GetObject<MobilityModel> ()->SetPosition (
          Vector (xMMTC->GetValue (), yMMTC->GetValue (), zMMTC->GetValue ()));
    }

  if (scenarioAB == "A")
    {
      // EMBB mobility (workers)
      // 12 round trips per hour per worker aproximately
      embbMobility.SetMobilityModel ("ns3::WaypointMobilityModel", "InitialPositionIsWaypoint",
                                     BooleanValue (false));

      Ptr<UniformRandomVariable> yInitialEMBB = CreateObject<UniformRandomVariable> ();
      yInitialEMBB->SetAttribute ("Min", DoubleValue (15));
      yInitialEMBB->SetAttribute ("Max", DoubleValue (28));

      for (uint32_t i = 0; i < embbNodes.GetN (); i++)
        {
          embbMobility.Install (embbNodes.Get (i));

          // Initial Position EMBB
          double xInitial = 39;
          double yInitial = double (yInitialEMBB->GetValue ());
          embbNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (embbNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 1)));

          //Workers mobility steps:
          // 1- Adjust Y axes to the rack randomly selected
          // 2- Adjust X axes to point 4 randomly selected
          // 3- Add Waypoints to GO from 5 to 4
          // 4- Add Waypoints to return from 4 to 5
          // 5- When the UE return to 5, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Points 4 -> 1 (x = 35 m), 2 (x = 30 m), 3 (x = 25 m)
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = embbNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION EMBB " << wpX << ", " << wpY << std::endl;

          UpdateEMBBWaypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX, wpY,
                               ueHeight, mob);
          umapUpdate.insert (std::make_pair (embbNodes.Get (i)->GetId (), false));
        }
    }
  else if (scenarioAB == "B")
    {
      // URLLC mobility (AGVs)
      urllcMobility.SetMobilityModel ("ns3::WaypointMobilityModel", "InitialPositionIsWaypoint",
                                      BooleanValue (false));

      // Mobility URLLC Type 1
      Ptr<UniformRandomVariable> xInitialURLLCT1 = CreateObject<UniformRandomVariable> ();
      xInitialURLLCT1->SetAttribute ("Min", DoubleValue (1));
      xInitialURLLCT1->SetAttribute ("Max", DoubleValue (7));

      Ptr<UniformRandomVariable> xInitialURLLCT2 = CreateObject<UniformRandomVariable> ();
      xInitialURLLCT2->SetAttribute ("Min", DoubleValue (8));
      xInitialURLLCT2->SetAttribute ("Max", DoubleValue (16));

      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          urllcMobility.Install (urllcNodes.Get (i));

          // Initial Position URLLCT1
          double xInitial = double (xInitialURLLCT1->GetValue ());
          double yInitial = 11;
          urllcNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (urllcNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 2)));

          //URLLC Type1 mobility steps:
          // 1- Adjust Y axes to y = 25 m
          // 2- Adjust X axes to point 1
          // 3- Add Waypoints to GO from 2 to 1
          // 4- Add Waypoints to return from 1 to 2
          // 5- When the UE return to 2, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Point 1 -> x = -6 m
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = urllcNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION URLLCT1 " << wpX << ", " << wpY << std::endl;

          UpdateURLLCT1Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX,
                                  wpY, ueHeight, mob);
          umapUpdate.insert (std::make_pair (urllcNodes.Get (i)->GetId (), false));
        }

      // Mobility URLLC Type 2
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          urllcMobility.Install (urllcNodes.Get (i));

          // Initial Position URLLCT2
          double xInitial = double (xInitialURLLCT2->GetValue ());
          double yInitial = 5.0;
          urllcNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (urllcNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 3)));

          //URLLC Type2 mobility steps:
          // 1- Adjust Y axes to the rack randomly selected
          // 2- Adjust X axes to point 4 randomly selected
          // 3- Add Waypoints to GO from 3 to 4
          // 4- Add Waypoints to return from 4 to 3
          // 5- When the UE return to 3, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Points 4 -> 1 (x = 35 m), 2 (x = 30 m), 3 (x = 25 m)
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = urllcNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION URLLCT2 " << wpX << ", " << wpY << std::endl;

          UpdateURLLCT2Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX,
                                  wpY, ueHeight, mob);
          umapUpdate.insert (std::make_pair (urllcNodes.Get (i)->GetId (), false));
        }
    }
  else
    { //Scenario C, workers and AGVs Type 1 and 2

      // EMBB mobility (workers)
      // 12 round trips per hour per worker aproximately
      embbMobility.SetMobilityModel ("ns3::WaypointMobilityModel", "InitialPositionIsWaypoint",
                                     BooleanValue (false));

      // Valor x inicial fijo, valor y inicial variable
      Ptr<UniformRandomVariable> yInitialEMBB = CreateObject<UniformRandomVariable> ();
      yInitialEMBB->SetAttribute ("Min", DoubleValue (15));
      yInitialEMBB->SetAttribute ("Max", DoubleValue (28));

      for (uint32_t i = 0; i < embbNodes.GetN (); i++)
        {
          embbMobility.Install (embbNodes.Get (i));

          // Initial Position EMBB
          double xInitial = 39;
          double yInitial = double (yInitialEMBB->GetValue ());
          embbNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (embbNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 1)));

          //Workers mobility steps:
          // 1- Adjust Y axes to the rack randomly selected
          // 2- Adjust X axes to point 4 randomly selected
          // 3- Add Waypoints to GO from 5 to 4
          // 4- Add Waypoints to return from 4 to 5
          // 5- When the UE return to 5, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Points 4 -> 1 (x = 35 m), 2 (x = 30 m), 3 (x = 25 m)
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = embbNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION EMBB " << wpX << ", " << wpY << std::endl;

          UpdateEMBBWaypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX, wpY,
                               ueHeight, mob);
          umapUpdate.insert (std::make_pair (embbNodes.Get (i)->GetId (), false));
        }

      // URLLC mobility (AGVs)
      urllcMobility.SetMobilityModel ("ns3::WaypointMobilityModel", "InitialPositionIsWaypoint",
                                      BooleanValue (false));

      // Mobility URLLC Type 1
      Ptr<UniformRandomVariable> xInitialURLLCT1 = CreateObject<UniformRandomVariable> ();
      xInitialURLLCT1->SetAttribute ("Min", DoubleValue (1));
      xInitialURLLCT1->SetAttribute ("Max", DoubleValue (7));

      Ptr<UniformRandomVariable> xInitialURLLCT2 = CreateObject<UniformRandomVariable> ();
      xInitialURLLCT2->SetAttribute ("Min", DoubleValue (8));
      xInitialURLLCT2->SetAttribute ("Max", DoubleValue (16));

      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          urllcMobility.Install (urllcNodes.Get (i));

          // Initial Position URLLCT1
          double xInitial = double (xInitialURLLCT1->GetValue ());
          double yInitial = 11;
          urllcNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (urllcNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 2)));

          //URLLC Type1 mobility steps:
          // 1- Adjust Y axes to y = 25 m
          // 2- Adjust X axes to point 1
          // 3- Add Waypoints to GO from 2 to 1
          // 4- Add Waypoints to return from 1 to 2
          // 5- When the UE return to 2, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Point 1 -> x = -6 m
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = urllcNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION URLLCT1 " << wpX << ", " << wpY << std::endl;

          UpdateURLLCT1Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX,
                                  wpY, ueHeight, mob);
          umapUpdate.insert (std::make_pair (urllcNodes.Get (i)->GetId (), false));
        }

      // Mobility URLLC Type 2
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          urllcMobility.Install (urllcNodes.Get (i));

          // Initial Position URLLCT2
          double xInitial = double (xInitialURLLCT2->GetValue ());
          double yInitial = 5.0;
          urllcNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (
              Vector (xInitial, yInitial, ueHeight));
          // Save initial position
          umap.insert (std::make_pair (urllcNodes.Get (i)->GetId (),
                                       std::make_pair (Vector (xInitial, yInitial, 1.5), 3)));

          //URLLC Type2 mobility steps:
          // 1- Adjust Y axes to the rack randomly selected
          // 2- Adjust X axes to point 4 randomly selected
          // 3- Add Waypoints to GO from 3 to 4
          // 4- Add Waypoints to return from 4 to 3
          // 5- When the UE return to 3, in CourseChange trace, when actualPosition==InitialPosition, do same steps, with a time waiting without movement.

          // Points 4 -> 1 (x = 35 m), 2 (x = 30 m), 3 (x = 25 m)
          double timeGranularity = 0.1;
          double wpTime = 0;
          double vel = 1; // m/s
          double xGranurality = vel * timeGranularity;
          double yGranurality = vel * timeGranularity;
          double wpX = xInitial; // Initial X value
          double wpY = yInitial; // Initial Y value
          Ptr<WaypointMobilityModel> mob = urllcNodes.Get (i)->GetObject<WaypointMobilityModel> ();

          //std::cout << "INITIAL POSITION URLLCT2 " << wpX << ", " << wpY << std::endl;

          UpdateURLLCT2Waypoints (timeGranularity, wpTime, vel, xGranurality, yGranurality, wpX,
                                  wpY, ueHeight, mob);
          umapUpdate.insert (std::make_pair (urllcNodes.Get (i)->GetId (), false));
        }
    }

  // gNBs position - fixed mobility
  gnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  bsPositionAlloc->Add (Vector (39.0, 1.0, gNbHeight));
  gnbMobility.SetPositionAllocator (bsPositionAlloc);
  gnbMobility.Install (gNbNodes);

  // ###########################################
  //File with UEs initial position (URLLC, MMTC, EMBB)
  std::ofstream m_outFileMMTCPosition;
  m_outFileMMTCPosition.open ("MMTC-Position.txt");
  m_outFileMMTCPosition.setf (std::ios_base::fixed);

  if (!m_outFileMMTCPosition.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << "MMTC-Position.txt");
    }

  for (uint32_t i = 0; i < mmtcNodes.GetN (); i++)
    {
      m_outFileMMTCPosition << mmtcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x
                            << "\t"
                            << mmtcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y
                            << "\t"
                            << mmtcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z
                            << std::endl;
    }
  m_outFileMMTCPosition.close ();

  if (scenarioAB == "A")
    {
      // EMBB initial position (workers)
      std::ofstream m_outFileEMBBPosition;
      m_outFileEMBBPosition.open ("EMBB-Position.txt");
      m_outFileEMBBPosition.setf (std::ios_base::fixed);

      if (!m_outFileEMBBPosition.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << "EMBB-Position.txt");
        }

      for (uint32_t i = 0; i < embbNodes.GetN (); i++)
        {
          m_outFileEMBBPosition << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x
                                << "\t"
                                << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y
                                << "\t"
                                << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z
                                << std::endl;
        }
      m_outFileEMBBPosition.close ();
    }
  else if (scenarioAB == "B")
    {
      // URLLC initial position (AGVs) - Type1 and Type2
      std::ofstream m_outFileURLLCPosition;
      m_outFileURLLCPosition.open ("URLLC-Position.txt");
      m_outFileURLLCPosition.setf (std::ios_base::fixed);

      if (!m_outFileURLLCPosition.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << "URLLC-Position.txt");
        }

      for (uint32_t i = 0; i < urllcNodes.GetN (); i++)
        {
          m_outFileURLLCPosition
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x << "\t"
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y << "\t"
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z << std::endl;
        }
      m_outFileURLLCPosition.close ();
    }
  else
    { //scenario C, workers and AGVs

      // EMBB initial position (workers)
      std::ofstream m_outFileEMBBPosition;
      m_outFileEMBBPosition.open ("EMBB-Position.txt");
      m_outFileEMBBPosition.setf (std::ios_base::fixed);

      if (!m_outFileEMBBPosition.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << "EMBB-Position.txt");
        }

      for (uint32_t i = 0; i < embbNodes.GetN (); i++)
        {
          m_outFileEMBBPosition << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x
                                << "\t"
                                << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y
                                << "\t"
                                << embbNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z
                                << std::endl;
        }
      m_outFileEMBBPosition.close ();

      // URLLC initial position (AGVs) - Type1 and Type2
      std::ofstream m_outFileURLLCPosition;
      m_outFileURLLCPosition.open ("URLLC-Position.txt");
      m_outFileURLLCPosition.setf (std::ios_base::fixed);

      if (!m_outFileURLLCPosition.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << "URLLC-Position.txt");
        }

      for (uint32_t i = 0; i < urllcNodes.GetN (); i++)
        {
          m_outFileURLLCPosition
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().x << "\t"
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().y << "\t"
              << urllcNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ().z << std::endl;
        }
      m_outFileURLLCPosition.close ();
    }

  /*----------------- BUILDING CONFIGURATION ----------------------------- */
  if (enableBuildings)
    {

      int numRacks = 7;
      int xminIzq = 25;
      int xmaxDer = 35;
      int ymin = 2;
      int ymax = 4;
      int zmin = 0;
      int zmax = 6;

      // Storage racks blocks
      for (int i = 0; i < numRacks; i++)
        {
          Ptr<Building> rack = Create<Building> ();
          rack->SetBoundaries (Box (xminIzq, xmaxDer, ymin, ymax, zmin, zmax));
          rack->SetBuildingType (Building::Office);
          rack->SetExtWallsType (Building::ConcreteWithoutWindows);
          rack->SetNFloors (1);
          rack->SetNRoomsX (1);
          rack->SetNRoomsY (1);

          ymin = ymin + 4;
          ymax = ymax + 4;
        }

      //Palletizing Machine building izq (point 2)
      Ptr<Building> palletizingMachineIzq = Create<Building> ();
      palletizingMachineIzq->SetBoundaries (Box (1, 6, 2, 10, 0, 6));
      palletizingMachineIzq->SetBuildingType (Building::Office);
      palletizingMachineIzq->SetExtWallsType (Building::ConcreteWithoutWindows);
      palletizingMachineIzq->SetNFloors (1);
      palletizingMachineIzq->SetNRoomsX (1);
      palletizingMachineIzq->SetNRoomsY (1);

      //palletizing Machine Der (point 3)
      Ptr<Building> palletizingMachineDer = Create<Building> ();
      palletizingMachineDer->SetBoundaries (Box (6.1, 16, 2, 4, 0, 6));
      palletizingMachineDer->SetBuildingType (Building::Office);
      palletizingMachineDer->SetExtWallsType (Building::ConcreteWithoutWindows);
      palletizingMachineDer->SetNFloors (1);
      palletizingMachineDer->SetNRoomsX (1);
      palletizingMachineDer->SetNRoomsY (1);

      BuildingsHelper::Install (gNbNodes);

      for (uint32_t i = 0; i < gNbNodes.GetN (); i++)
        {
          gNbNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
              gNbNodes.Get (i)->GetObject<MobilityModel> ());
        }

      BuildingsHelper::Install (mmtcNodes);
      // MMTC
      for (uint32_t i = 0; i < mmtcNodes.GetN (); i++)
        {
          mmtcNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
              mmtcNodes.Get (i)->GetObject<MobilityModel> ());
        }

      if (scenarioAB == "A")
        {
          // MMTC mobility (smart tags) -> Fixed in a random position in the racks
          // EMBB mobility (workers)
          BuildingsHelper::Install (embbNodes);
          for (uint32_t i = 0; i < embbNodes.GetN (); i++)
            {
              embbNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
                  embbNodes.Get (i)->GetObject<MobilityModel> ());
            }
        }
      else if (scenarioAB == "B")
        {
          // MMTC mobility (smart tags) -> Fixed in a random position in the racks
          // URLLC (AGVs) - Type1 and Type2
          BuildingsHelper::Install (urllcNodes);
          for (uint32_t i = 0; i < urllcNodes.GetN (); i++)
            {
              urllcNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
                  urllcNodes.Get (i)->GetObject<MobilityModel> ());
            }
        }
      else
        { //scenario C, workers and AGVs

          // EMBB mobility (workers)
          BuildingsHelper::Install (embbNodes);
          for (uint32_t i = 0; i < embbNodes.GetN (); i++)
            {
              embbNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
                  embbNodes.Get (i)->GetObject<MobilityModel> ());
            }

          // URLLC (AGVs) - Type1 and Type2
          BuildingsHelper::Install (urllcNodes);
          for (uint32_t i = 0; i < urllcNodes.GetN (); i++)
            {
              urllcNodes.Get (i)->GetObject<MobilityBuildingInfo> ()->MakeConsistent (
                  urllcNodes.Get (i)->GetObject<MobilityModel> ());
            }
        }
      //BuildingsHelper::MakeMobilityModelConsistent (); warning-deprecated

      TraceBuildingLoc ();
      std::cout << "Buildings configured" << std::endl;
    }
  /* --------------------------- BUILDING END ------------------------ */

  // Install nr net devices
  NetDeviceContainer gNbNetDev;
  gNbNetDev.Add (nrHelper->InstallGnbDevice (gNbNodes.Get (0), allBwps)); // gNB0

  NetDeviceContainer mmtcUeNetDev, embbUeNetDev, urllcUeNetDev;

  // MMTC
  for (uint32_t i = 0; i < mmtcUEs; i++)
    {
      mmtcUeNetDev.Add (nrHelper->InstallUeDevice (mmtcNodes.Get (i), allBwps));
    }

  if (scenarioAB == "A")
    {
      // EMBB (workers)
      for (uint32_t i = 0; i < embbUEs; i++)
        {
          embbUeNetDev.Add (nrHelper->InstallUeDevice (embbNodes.Get (i), allBwps));
        }
    }
  else if (scenarioAB == "B")
    {
      // URLLC (AGVs) - Type1
      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          urllcUeNetDev.Add (nrHelper->InstallUeDevice (urllcNodes.Get (i), allBwps));
        }
      // Type2
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          urllcUeNetDev.Add (nrHelper->InstallUeDevice (urllcNodes.Get (i), allBwps));
        }
    }
  else
    {
      // EMBB (workers)
      for (uint32_t i = 0; i < embbUEs; i++)
        {
          embbUeNetDev.Add (nrHelper->InstallUeDevice (embbNodes.Get (i), allBwps));
        }

      // URLLC (AGVs) - Type1
      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          urllcUeNetDev.Add (nrHelper->InstallUeDevice (urllcNodes.Get (i), allBwps));
        }
      // Type2
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          urllcUeNetDev.Add (nrHelper->InstallUeDevice (urllcNodes.Get (i), allBwps));
        }
    }

  //int64_t randomStream = 1;
  randomStream += nrHelper->AssignStreams (gNbNetDev, randomStream);
  randomStream += nrHelper->AssignStreams (mmtcUeNetDev, randomStream);
  if (scenarioAB == "A")
    {
      randomStream += nrHelper->AssignStreams (embbUeNetDev, randomStream);
    }
  else if (scenarioAB == "B")
    {
      randomStream += nrHelper->AssignStreams (urllcUeNetDev, randomStream);
    }
  else
    {
      randomStream += nrHelper->AssignStreams (embbUeNetDev, randomStream);
      randomStream += nrHelper->AssignStreams (urllcUeNetDev, randomStream);
    }

  //NS_ASSERT (gNbNetDev.GetN () == 4);

  // -------------- First GNB:

  // BWP0, MMTC
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 0)->SetAttribute ("Numerology", UintegerValue (0));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 0)
      ->SetAttribute ("Pattern", StringValue ("F|F|F|F|F|F|F|F|F|F|"));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (txPower));

  // BWP1, EMBB
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 1)->SetAttribute ("Numerology", UintegerValue (0));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 1)
      ->SetAttribute ("Pattern", StringValue ("F|F|F|F|F|F|F|F|F|F|"));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 1)->SetAttribute ("TxPower", DoubleValue (txPower));

  // BWP2, URLLC
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 2)
      ->SetAttribute ("Numerology", UintegerValue (numerology));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 2)
      ->SetAttribute ("Pattern", StringValue ("F|F|F|F|F|F|F|F|F|F|"));
  nrHelper->GetGnbPhy (gNbNetDev.Get (0), 2)->SetAttribute ("TxPower", DoubleValue (txPower));

  // Link the two FDD BWP:
  //nrHelper->GetBwpManagerGnb (gNbNetDev.Get (0))->SetOutputLink (2, 1);

  // Set the UE routing:

  //for (uint32_t i = 0; i < ueNetDev.GetN (); i++)
  //  {
  //    nrHelper->GetBwpManagerUe (ueNetDev.Get (i))->SetOutputLink (1, 2);
  //  }

  // When all the configuration is done, explicitly call UpdateConfig ()

  for (auto it = gNbNetDev.Begin (); it != gNbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = mmtcUeNetDev.Begin (); it != mmtcUeNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  if (scenarioAB == "A")
    {
      for (auto it = embbUeNetDev.Begin (); it != embbUeNetDev.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }
    }
  else if (scenarioAB == "B")
    {
      for (auto it = urllcUeNetDev.Begin (); it != urllcUeNetDev.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }
    }
  else // scenario C, Workers and AGVs
    {
      for (auto it = embbUeNetDev.Begin (); it != embbUeNetDev.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }

      for (auto it = urllcUeNetDev.Begin (); it != urllcUeNetDev.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }
    }

  // From here, it is standard NS3. In the future, we will create helpers
  // for this part as well.

  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // connect a remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  internet.Install (mmtcNodes);

  if (scenarioAB == "A")
    {
      // EMBB (workers)
      internet.Install (embbNodes);
    }
  else if (scenarioAB == "B")
    {
      // URLLC (AGVs) - Type1 and Type2
      internet.Install (urllcNodes);
    }
  else
    {
      // EMBB (workers) and AGVs
      internet.Install (embbNodes);
      internet.Install (urllcNodes);
    }

  Ipv4InterfaceContainer mmtcUeIpIface, embbUeIpIface, urllcUeIpIface;
  mmtcUeIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mmtcUeNetDev));
  if (scenarioAB == "A")
    {
      // EMBB (workers)
      embbUeIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (embbUeNetDev));
    }
  else if (scenarioAB == "B")
    {
      // URLLC (AGVs) - Type1 and Type2
      urllcUeIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (urllcUeNetDev));
    }
  else
    { // workers and AGVs
      embbUeIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (embbUeNetDev));
      urllcUeIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (urllcUeNetDev));
    }

  // Set the default gateway for the UEs
  for (uint32_t j = 0; j < mmtcNodes.GetN (); ++j)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (mmtcNodes.Get (j)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  if (scenarioAB == "A")
    {
      // EMBB (workers)
      for (uint32_t j = 0; j < embbNodes.GetN (); ++j)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (embbNodes.Get (j)->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }
    }
  else if (scenarioAB == "B")
    {
      // URLLC (AGVs) - Type1 and Type2
      for (uint32_t j = 0; j < urllcNodes.GetN (); ++j)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (urllcNodes.Get (j)->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }
    }
  else
    { //workers and AGVs
      for (uint32_t j = 0; j < embbNodes.GetN (); ++j)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (embbNodes.Get (j)->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }

      for (uint32_t j = 0; j < urllcNodes.GetN (); ++j)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (urllcNodes.Get (j)->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }
    }

  // Attach MMTC Devices to gNB
  for (uint32_t i = 0; i < mmtcUeNetDev.GetN (); i++)
    {
      nrHelper->AttachToEnb (mmtcUeNetDev.Get (i), gNbNetDev.Get (0));
    }

  if (scenarioAB == "A")
    {
      // Attach EMBB Devices to gNB
      for (uint32_t i = 0; i < embbUeNetDev.GetN (); i++)
        {
          nrHelper->AttachToEnb (embbUeNetDev.Get (i), gNbNetDev.Get (0));
        }
    }
  else if (scenarioAB == "B")
    {
      // Attach URLLC Type 1 Devices to gNB
      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          nrHelper->AttachToEnb (urllcUeNetDev.Get (i), gNbNetDev.Get (0));
        }

      // Attach URLLC Type 2 Devices to gNB
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          nrHelper->AttachToEnb (urllcUeNetDev.Get (i), gNbNetDev.Get (0));
        }
    }
  else
    {
      // Attach EMBB Devices to gNB
      for (uint32_t i = 0; i < embbUeNetDev.GetN (); i++)
        {
          nrHelper->AttachToEnb (embbUeNetDev.Get (i), gNbNetDev.Get (0));
        }

      // Attach URLLC Type 1 Devices to gNB
      for (uint32_t i = 0; i < urllcUEsType1; i++)
        {
          nrHelper->AttachToEnb (urllcUeNetDev.Get (i), gNbNetDev.Get (0));
        }

      // Attach URLLC Type 2 Devices to gNB
      for (uint32_t i = urllcUEsType1; i < urllcUEsType1 + urllcUEsType2; i++)
        {
          nrHelper->AttachToEnb (urllcUeNetDev.Get (i), gNbNetDev.Get (0));
        }
    }

  std::cout << "UEs are attached to the network" << std::endl;

  //////////////////////// APPLICATIONS TRAFFIC SET UP //////////////////////////////////
  ApplicationContainer mmtcClientApps, embbClientApps, urllcClientApps;
  ApplicationContainer mmtcServerApps, embbServerApps, urllcServerApps;
  uint16_t ulPortMMTC = 2000;
  uint16_t dlPortEMBB = 4000;
  uint16_t dlPortURLLC = 5000;
  uint8_t remotehostAppNum = 0;

  // The bearer that will carry MMTC traffic
  EpsBearer MMTCBearer (EpsBearer::GBR_GAMING);
  // The filter for the MMTC traffic
  /*Ptr<EpcTft> MMTCTft = Create<EpcTft> ();
	EpcTft::PacketFilter ulpfMMTC;
	ulpfMMTC.remotePortStart = ulPortMMTC;
	ulpfMMTC.remotePortEnd = ulPortMMTC;
	ulpfMMTC.direction = EpcTft::UPLINK;
	MMTCTft->Add (ulpfMMTC); */

  // The bearer that will carry eMBB traffic
  EpsBearer eMBBBearer (EpsBearer::GBR_CONV_VOICE);
  // The filter for the eMBB traffic
  Ptr<EpcTft> eMBBTft = Create<EpcTft> ();
  EpcTft::PacketFilter dlpfeMBB;
  dlpfeMBB.localPortStart = dlPortEMBB;
  dlpfeMBB.localPortEnd = dlPortEMBB;
  eMBBTft->Add (dlpfeMBB);

  // The bearer that will carry URLLC traffic
  EpsBearer URLLCBearer (EpsBearer::GBR_CONV_VIDEO);
  // The filter for the URLLC traffic
  Ptr<EpcTft> URLLCTft = Create<EpcTft> ();
  EpcTft::PacketFilter dlpfURLLC;
  dlpfURLLC.localPortStart = dlPortURLLC;
  dlpfURLLC.localPortEnd = dlPortURLLC;
  URLLCTft->Add (dlpfURLLC);

  // MMTC Application
  for (uint32_t j = 0; j < mmtcNodes.GetN (); j++)
    {
      //++ulPortMMTC;

      // UL traffic mMTC
      UdpServerHelper mmtcUlPacketSinkHelper (ulPortMMTC);
      UdpClientHelper mmtcUlClient (remoteHostAddr, ulPortMMTC);

      mmtcUlClient.SetAttribute ("Interval", TimeValue (Seconds (mmtcInterval)));
      mmtcUlClient.SetAttribute ("PacketSize", UintegerValue (mmtcPacketSize));
      mmtcUlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      mmtcUlClient.SetAttribute ("EnableRandom", UintegerValue (1));
      mmtcUlClient.SetAttribute ("TimeGranularity",
                                 UintegerValue (0)); //0 Value in seconds, 1- Value in minutes
      mmtcUlClient.SetAttribute ("UniformMinValue", DoubleValue (1.5));
      mmtcUlClient.SetAttribute ("UniformMaxValue", DoubleValue (30.0));
      //Repetitions for each packet transmission
      mmtcUlClient.SetAttribute ("Repetitions", UintegerValue (mmtcRepetitions));

      mmtcClientApps.Add (mmtcUlClient.Install (mmtcNodes.Get (j)));
      mmtcServerApps.Add (mmtcUlPacketSinkHelper.Install (remoteHost));
      remotehostAppNum++;

      Ptr<EpcTft> MMTCTft = Create<EpcTft> ();
      EpcTft::PacketFilter ulpfMMTC;
      ulpfMMTC.remotePortStart = ulPortMMTC;
      ulpfMMTC.remotePortEnd = ulPortMMTC;
      ulpfMMTC.direction = EpcTft::UPLINK;
      MMTCTft->Add (ulpfMMTC);
      /*ulpfMMTC.localPortStart = ulPortMMTC;
  		ulpfMMTC.localPortEnd = ulPortMMTC;
  		MMTCTft->Add (ulpfMMTC); */
      nrHelper->ActivateDedicatedEpsBearer (mmtcUeNetDev.Get (j), MMTCBearer, MMTCTft);
      ulPortMMTC++;
    }

  if (scenarioAB == "A")
    {
      // EMBB application (workers) (DL)
      for (uint32_t j = 0; j < embbNodes.GetN (); ++j)
        {
          //++dlPortEMBB;
          /*PacketSinkHelper embbSinkDL ("ns3::UdpSocketFactory",
  																	InetSocketAddress (Ipv4Address::GetAny (), dlPort));
  				//DL
  				OnOffHelper embbSourceDL ("ns3::UdpSocketFactory",
  														 InetSocketAddress (embbUeIpIface.GetAddress (j), dlPort));

  				// Set the amount of data to send in bytes.  Zero is unlimited.
  				embbSourceDL.SetAttribute ("PacketSize", UintegerValue (1000)); // bytes
  				embbSourceDL.SetAttribute ("DataRate", DataRateValue (DataRate ("50Mb/s")));
  	 			embbSourceDL.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

  				embbClientApps.Add (embbSourceDL.Install (remoteHost));
  			 	embbServerApps.Add (embbSinkDL.Install (embbNodes.Get (j))); */

          // DL Traffic
          UdpServerHelper embbDlPacketSinkHelper (dlPortEMBB);
          UdpClientHelper embbDlClient (embbUeIpIface.GetAddress (j), dlPortEMBB);

          double bitRate =
              4000000; // 75 Mbps will saturate the NR system of 20 MHz with the NrEesmIrT1 error model
          double udpPacketSize = 1000;
          double lambda = bitRate / static_cast<double> (udpPacketSize * 8);

          embbDlClient.SetAttribute ("Interval", TimeValue (Seconds (1.0 / lambda)));
          embbDlClient.SetAttribute ("PacketSize", UintegerValue (udpPacketSize));
          embbDlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));

          embbClientApps.Add (embbDlPacketSinkHelper.Install (embbNodes.Get (j)));
          embbServerApps.Add (embbDlClient.Install (remoteHost));
          remotehostAppNum++;

          /*dlpfeMBB.localPortStart = dlPortEMBB;
  				dlpfeMBB.localPortEnd = dlPortEMBB;
  				eMBBTft->Add (dlpfeMBB);*/
          nrHelper->ActivateDedicatedEpsBearer (embbUeNetDev.Get (j), eMBBBearer, eMBBTft);
        }
    }
  else if (scenarioAB == "B")
    {
      // URLLC Application- // 10 messages/second (interval 100 ms)
      for (uint32_t j = 0; j < urllcNodes.GetN (); ++j)
        {
          //++dlPortURLLC;

          // DL Traffic
          UdpServerHelper urllcDlPacketSinkHelper (dlPortURLLC);
          UdpClientHelper urllcDlClient (urllcUeIpIface.GetAddress (j), dlPortURLLC);

          urllcDlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (urllcInterval)));
          urllcDlClient.SetAttribute ("PacketSize", UintegerValue (urllcPacketSize));
          urllcDlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));

          urllcClientApps.Add (urllcDlPacketSinkHelper.Install (urllcNodes.Get (j)));
          urllcServerApps.Add (urllcDlClient.Install (remoteHost));
          umapAppRemoteHost.insert (
              std::make_pair (urllcNodes.Get (j)->GetId (), remotehostAppNum));
          remotehostAppNum++;

          /*dlpfURLLC.localPortStart = dlPortURLLC;
  			dlpfURLLC.localPortEnd = dlPortURLLC;
  			URLLCTft->Add (dlpfURLLC);*/
          nrHelper->ActivateDedicatedEpsBearer (urllcUeNetDev.Get (j), URLLCBearer, URLLCTft);
        }
    }
  else //workers and AGVs
    {
      // EMBB application (workers) (DL)
      for (uint32_t j = 0; j < embbNodes.GetN (); ++j)
        {
          //++dlPortEMBB;

          // DL Traffic
          UdpServerHelper embbDlPacketSinkHelper (dlPortEMBB);
          UdpClientHelper embbDlClient (embbUeIpIface.GetAddress (j), dlPortEMBB);

          double bitRate =
              4000000; // 75 Mbps will saturate the NR system of 20 MHz with the NrEesmIrT1 error model
          double udpPacketSize = 1000;
          double lambda = bitRate / static_cast<double> (udpPacketSize * 8);

          embbDlClient.SetAttribute ("Interval", TimeValue (Seconds (1.0 / lambda)));
          embbDlClient.SetAttribute ("PacketSize", UintegerValue (udpPacketSize));
          embbDlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));

          embbClientApps.Add (embbDlPacketSinkHelper.Install (embbNodes.Get (j)));
          embbServerApps.Add (embbDlClient.Install (remoteHost));
          remotehostAppNum++;

          /*dlpfeMBB.localPortStart = dlPortEMBB;
  				dlpfeMBB.localPortEnd = dlPortEMBB;
  				eMBBTft->Add (dlpfeMBB);*/
          nrHelper->ActivateDedicatedEpsBearer (embbUeNetDev.Get (j), eMBBBearer, eMBBTft);
        }

      // URLLC Application- // 10 messages/second (interval 100 ms)
      for (uint32_t j = 0; j < urllcNodes.GetN (); ++j)
        {
          //++dlPortURLLC;

          // DL Traffic
          UdpServerHelper urllcDlPacketSinkHelper (dlPortURLLC);
          UdpClientHelper urllcDlClient (urllcUeIpIface.GetAddress (j), dlPortURLLC);

          urllcDlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (urllcInterval)));
          urllcDlClient.SetAttribute ("PacketSize", UintegerValue (urllcPacketSize));
          urllcDlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));

          urllcClientApps.Add (urllcDlPacketSinkHelper.Install (urllcNodes.Get (j)));
          urllcServerApps.Add (urllcDlClient.Install (remoteHost));
          umapAppRemoteHost.insert (
              std::make_pair (urllcNodes.Get (j)->GetId (), remotehostAppNum));
          remotehostAppNum++;

          /*dlpfURLLC.localPortStart = dlPortURLLC;
  				dlpfURLLC.localPortEnd = dlPortURLLC;
  				URLLCTft->Add (dlpfURLLC);*/
          nrHelper->ActivateDedicatedEpsBearer (urllcUeNetDev.Get (j), URLLCBearer, URLLCTft);
        }
    }

  // Application Start and Stop
  // MMTC
  mmtcClientApps.Start (Seconds (udpAppStartTime));
  mmtcClientApps.Stop (Seconds (simTime - 1));
  mmtcServerApps.Start (Seconds (udpAppStartTime));
  mmtcServerApps.Stop (Seconds (simTime));
  // EMBB
  embbClientApps.Start (Seconds (udpAppStartTime));
  embbClientApps.Stop (Seconds (simTime - 1));
  embbServerApps.Start (Seconds (udpAppStartTime));
  embbServerApps.Stop (Seconds (simTime));
  // URLLC
  urllcClientApps.Start (Seconds (udpAppStartTime));
  urllcClientApps.Stop (Seconds (simTime - 1));
  urllcServerApps.Start (Seconds (udpAppStartTime));
  urllcServerApps.Stop (Seconds (simTime));

  // enable the traces provided by the nr module
  nrHelper->EnableTracesForANodeDevice ("*", "*");

  //Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::UdpServer/RxWithAddresses", MakeCallback (&RxPacketServer));
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));

  //Create a file with IP associate to type of Service (MMTC, EMBB, URLLC)
  std::unordered_map<uint32_t, uint8_t> umapIP;
  std::ofstream m_outFileIP;
  m_outFileIP.open ("IP-Table.txt");
  m_outFileIP.setf (std::ios_base::fixed);

  if (!m_outFileIP.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << "IP-Table.txt");
    }

  m_outFileIP << "IPAddress" << "\t" << "Service" << std::endl;

  for (uint32_t j = 0; j < mmtcNodes.GetN (); ++j)
    {
      m_outFileIP << mmtcUeIpIface.GetAddress (j) << "\t" << "MMTC" << std::endl;
      umapIP.insert (std::make_pair (mmtcUeIpIface.GetAddress (j).Get (), 1));
    }

  for (uint32_t j = 0; j < embbNodes.GetN (); ++j)
    {
      m_outFileIP << embbUeIpIface.GetAddress (j) << "\t" << "EMBB" << std::endl;
      umapIP.insert (std::make_pair (embbUeIpIface.GetAddress (j).Get (), 2));
    }

  for (uint32_t j = 0; j < urllcNodes.GetN (); ++j)
    {
      m_outFileIP << urllcUeIpIface.GetAddress (j) << "\t" << "URLLC" << std::endl;
      umapIP.insert (std::make_pair (urllcUeIpIface.GetAddress (j).Get (), 3));
    }
  m_outFileIP.close ();

  FlowMonitorHelper flowmonHelper;
  NodeContainer endpointNodes;
  endpointNodes.Add (remoteHost);
  endpointNodes.Add (mmtcNodes);
  endpointNodes.Add (embbNodes);
  endpointNodes.Add (urllcNodes);

  Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install (endpointNodes);
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));

  std::cout << "######### Simulator::Run() ###########" << std::endl;
  std::cout << std::endl;

  m_timeEvent = Simulator::Schedule (MilliSeconds (100), &PrintSimulationTime, simTime);
  Simulator::Schedule (Seconds (1), &PrintIMSI_RNTI, mmtcUeNetDev, embbUeNetDev, urllcUeNetDev);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  std::cout << "Simulation progress: " << "100 %" << std::endl << std::endl;

  m_timeEvent.Cancel ();

  //SaveTimePositions ("timePositions.txt",g_timeposition);
  m_courseChangeFile.close ();

  // Print per-flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

  double averageFlowThroughputMMTC = 0.0;
  double averageFlowDelayMMTC = 0.0;
  double numMMTCFlows = 0.0;

  double averageFlowThroughputEMBB = 0.0;
  double averageFlowDelayEMBB = 0.0;
  double numEMBBFlows = 0.0;

  double averageFlowThroughputURLLC = 0.0;
  double averageFlowDelayURLLC = 0.0;
  double numURLLCFlows = 0.0;

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
              << i->second.txBytes * 8.0 /
                     (i->second.timeLastTxPacket.GetSeconds () -
                      i->second.timeFirstTxPacket.GetSeconds ()) /
                     1000 / 1000
              << " Mbps\n";
      outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
      if (i->second.rxPackets > 0)
        {
          // Measure the duration of the flow from receiver's perspective
          double rxDuration =
              i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ();
          //double rxDuration = simTime - udpAppStartTime;
          switch (umapIP[t.destinationAddress.Get ()]) // (DL traffic)
            {
            case 2:
              averageFlowThroughputEMBB += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
              averageFlowDelayEMBB += 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;
              numEMBBFlows += 1.0;
              break;
            case 3:
              averageFlowThroughputURLLC += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
              averageFlowDelayURLLC +=
                  1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;
              numURLLCFlows += 1.0;
              break;
            default:
              //MMTC case (UL)
              if (umapIP[t.sourceAddress.Get ()] == 1)
                {
                  averageFlowThroughputMMTC += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
                  averageFlowDelayMMTC +=
                      1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;
                  numMMTCFlows += 1.0;
                }
              break;
            }

          //averageFlowThroughput += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
          //averageFlowDelay += 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;

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

  outFile << "\n\n  Mean flow throughput MMTC: " << averageFlowThroughputMMTC / numMMTCFlows
          << " Mbps\n";
  outFile << "  Mean flow delay MMTC: " << averageFlowDelayMMTC / numMMTCFlows << " ms\n";
  outFile << "\n\n  Mean flow throughput EMBB: " << averageFlowThroughputEMBB / numEMBBFlows
          << " Mbps\n";
  outFile << "  Mean flow delay EMBB: " << averageFlowDelayEMBB / numEMBBFlows << " ms\n";
  outFile << "\n\n  Mean flow throughput URLLC: " << averageFlowThroughputURLLC / numURLLCFlows
          << " Mbps\n";
  outFile << "  Mean flow delay URLLC: " << averageFlowDelayURLLC / numURLLCFlows << " ms\n";
  outFile.close ();

  Simulator::Destroy ();
  return 0;
}
