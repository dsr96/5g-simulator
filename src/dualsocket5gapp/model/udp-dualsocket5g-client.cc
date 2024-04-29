/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright 2007 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <ns3/fatal-error.h>
#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/nr-module.h"
#include "ns3/callback.h"
#include "udp-dualsocket5g-client.h"
#include "ns3/unique-packet-id-tag.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UdpDualSocket5gClientApplication");

NS_OBJECT_ENSURE_REGISTERED (UdpDualSocket5gClient);

TypeId
UdpDualSocket5gClient::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UdpDualSocket5gClient")
    .SetParent<Application> ()
    .SetGroupName("Applications")
     .AddConstructor<UdpDualSocket5gClient> ()
    .AddAttribute ("MaxPackets",
                   "The maximum number of packets the application will send",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpDualSocket5gClient::m_count),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Interval",
                   "The time to wait between packets",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&UdpDualSocket5gClient::m_interval),
                   MakeTimeChecker ())
    .AddAttribute ("RemoteAddress",
                   "The destination Address of the outbound packets",
                   AddressValue (),
                   MakeAddressAccessor (&UdpDualSocket5gClient::m_peerAddress),
                   MakeAddressChecker ())
    .AddAttribute ("RemotePort",
                   "The destination port of the outbound packets",
                   UintegerValue (0),
                   MakeUintegerAccessor (&UdpDualSocket5gClient::m_peerPort),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketSize", "Size of echo data in outbound packets",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpDualSocket5gClient::SetDataSize,
                                         &UdpDualSocket5gClient::GetDataSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SinrThreshold", "SINR threshold value in (dB) for gNB. Below the value, turn on packet duplication",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&UdpDualSocket5gClient::m_sinrThreshold),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("AlwaysDirectConnection", "Tx always via Direct Connection",
                   BooleanValue (false),
                   MakeBooleanAccessor (&UdpDualSocket5gClient::m_alwaysDirectConnection),
                   MakeBooleanChecker ())
    .AddAttribute ("AlwaysIndirectConnection", "Tx always via Indirect Connection",
                   BooleanValue (false),
                   MakeBooleanAccessor (&UdpDualSocket5gClient::m_alwaysIndirectConnection),
                   MakeBooleanChecker ())
    .AddAttribute ("AlwaysBothConnection", "Tx always via both connection",
                   BooleanValue (false),
                   MakeBooleanAccessor (&UdpDualSocket5gClient::m_alwaysBothConnection),
                   MakeBooleanChecker())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&UdpDualSocket5gClient::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx", "A packet has been received",
                     MakeTraceSourceAccessor (&UdpDualSocket5gClient::m_rxTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("TxWithAddresses", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&UdpDualSocket5gClient::m_txTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
    .AddTraceSource ("RxWithAddresses", "A packet has been received",
                     MakeTraceSourceAccessor (&UdpDualSocket5gClient::m_rxTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
  ;
  return tid;
}

UdpDualSocket5gClient::UdpDualSocket5gClient ()
{
  NS_LOG_FUNCTION (this);
  m_sent = 0;
  m_socketDirect = 0;
  m_socketIndirect = 0;
  m_netDeviceDirect = 0;
  m_netDeviceIndirect = 0;
  m_sendEvent = EventId ();
  m_data = 0;
  m_dataSize = 0;
  m_nrSinrValueReceived = 30.0; //Good SINR until the variable is set by SINR callback
  m_inputData.averageDirectRtt = 0;
  m_inputData.averageIndirectRtt = 0;
  m_inputData.SINRDirect = 30.0;
  m_inputData.SINRIndirect = 0.0;
  m_updateInputsPeriod = MilliSeconds(250);
}

UdpDualSocket5gClient::~UdpDualSocket5gClient()
{
  NS_LOG_FUNCTION (this);
  m_socketDirect = 0;
  m_socketIndirect = 0;

  delete [] m_data;
  m_data = 0;
  m_dataSize = 0;
}

void
UdpDualSocket5gClient::SetUpdateInputsPeriod (Time T)
{
	m_updateInputsPeriod = T;
}

void
UdpDualSocket5gClient::SetRemote (Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = ip;
  m_peerPort = port;
}

void
UdpDualSocket5gClient::SetRemote (Ipv4Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = Address (ip);
  m_peerPort = port;
}

void
UdpDualSocket5gClient::SetRemote (Ipv6Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = Address (ip);
  m_peerPort = port;
}

void
UdpDualSocket5gClient::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
UdpDualSocket5gClient::UpdateInputsForEvaluation (void)
{
  NS_LOG_FUNCTION (this);

  if (m_alwaysDirectConnection){
    connectionSelected = "DIRECT";
  } else if (m_alwaysIndirectConnection) {
    connectionSelected = "INDIRECT";
  } else if (m_alwaysBothConnection){
    connectionSelected = "BOTH";
  } else {
    //Evaluate condition in order to send via Direct, Indirect or both interfaces (PD)
    //double margin = 5.0;
    if ((m_inputData.SINRDirect > m_inputData.SINRIndirect)) {
      connectionSelected = "DIRECT";
    } else {
      connectionSelected = "INDIRECT";
    }

    int margin  = 5;
    if ((std::abs (m_inputData.SINRDirect - m_inputData.SINRIndirect) < margin) && ((m_inputData.SINRDirect < m_sinrThreshold) || (m_inputData.SINRIndirect < m_sinrThreshold))) {
      connectionSelected = "BOTH";
    }
  }

  Simulator::Schedule(m_updateInputsPeriod, &UdpDualSocket5gClient::UpdateInputsForEvaluation, this);
}

void
UdpDualSocket5gClient::StartApplication (void)
{
	NS_LOG_FUNCTION (this);

	Ptr<Node> node = GetNode();

//	for (uint k = 0; k < node->GetNDevices(); k++){

		Ptr<NrUeNetDevice> dev1 = node->GetDevice(0)->GetObject<NrUeNetDevice>();
		Ptr<NrUeNetDevice> dev2 = node->GetDevice(1)->GetObject<NrUeNetDevice>();

		if (dev1)
		m_netDeviceDirect = node->GetDevice(0)->GetObject<NrUeNetDevice>();

		if (dev2)
		m_netDeviceIndirect = node->GetDevice(1)->GetObject<NrUeNetDevice>();
//	}

	Ptr<Ipv4> ipstack4 = GetNode()->GetObject<Ipv4>();

	uint32_t idc = ipstack4->GetInterfaceForDevice(m_netDeviceDirect);
	uint32_t iic = ipstack4->GetInterfaceForDevice(m_netDeviceIndirect);

	if (m_netDeviceDirect)
		NS_LOG_INFO("Direct: " << m_netDeviceDirect->GetAddress() << " assigned to Iface " << idc << " (" << (ipstack4->GetAddress(1,0)).GetLocal() << ")");
	else
		NS_LOG_INFO("There is no direct connection!");

	if(m_netDeviceIndirect)
		NS_LOG_INFO("Indirect: " << m_netDeviceIndirect->GetAddress() << " assigned to Iface " << iic << " (" << (ipstack4->GetAddress(2,0)).GetLocal() << ")");
	else
		NS_LOG_INFO("There is no indirect connection!");

	//**************************
	//	IP <-> Socket binding
	//**************************

	if (m_socketDirect == 0 && m_netDeviceDirect)
	{
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		m_socketDirect = Socket::CreateSocket (GetNode (), tid);
		if (Ipv4Address::IsMatchingType(m_peerAddress) == true)
		{
			m_socketDirect->Bind((ipstack4->GetAddress(1,0)).GetLocal());
			NS_LOG_INFO("Binding direct to " <<  (ipstack4->GetAddress(1,0)).GetLocal() << " DEV: " << m_netDeviceDirect->GetAddress());
		  m_socketDirect->BindToNetDevice(m_netDeviceDirect);
			m_socketDirect->Connect (InetSocketAddress (Ipv4Address::ConvertFrom(m_peerAddress), m_peerPort));
		}

		m_socketDirect->SetRecvCallback (MakeCallback (&UdpDualSocket5gClient::HandleReadDirect, this));
	}

	if (m_socketIndirect == 0 && m_netDeviceIndirect)
	{
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		m_socketIndirect = Socket::CreateSocket (GetNode (), tid);
		if (Ipv4Address::IsMatchingType(m_peerAddress) == true)
		{
			m_socketIndirect->Bind((ipstack4->GetAddress(2,0)).GetLocal());
			NS_LOG_INFO("Binding indirect to " <<  (ipstack4->GetAddress(2,0)).GetLocal() << " DEV: " << m_netDeviceIndirect->GetAddress());
			m_socketIndirect->BindToNetDevice(m_netDeviceIndirect);
			m_socketIndirect->Connect (InetSocketAddress (Ipv4Address::ConvertFrom(m_peerAddress), m_peerPort));
		}

		m_socketIndirect->SetRecvCallback (MakeCallback (&UdpDualSocket5gClient::HandleReadIndirect, this));
	}

  Simulator::ScheduleNow(&UdpDualSocket5gClient::UpdateInputsForEvaluation, this);

	ScheduleTransmit (Seconds (0.));
}

void
UdpDualSocket5gClient::StopApplication ()
{
  NS_LOG_FUNCTION (this);

  if (m_socketIndirect != 0)
    {
      m_socketIndirect->Close ();
      m_socketIndirect->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_socketIndirect = 0;
    }

  if (m_socketDirect != 0)
    {
      m_socketDirect->Close ();
      m_socketDirect->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_socketDirect = 0;
    }

  Simulator::Cancel (m_sendEvent);
}

void
UdpDualSocket5gClient::SetDataSize (uint32_t dataSize)
{
  NS_LOG_FUNCTION (this << dataSize);

  //
  // If the client is setting the echo packet data size this way, we infer
  // that she doesn't care about the contents of the packet at all, so
  // neither will we.
  //
  delete [] m_data;
  m_data = 0;
  m_dataSize = 0;
  m_size = dataSize;
}

uint32_t
UdpDualSocket5gClient::GetDataSize (void) const
{
  NS_LOG_FUNCTION (this);
  return m_size;
}

void
UdpDualSocket5gClient::SetFill (std::string fill)
{
  NS_LOG_FUNCTION (this << fill);

  uint32_t dataSize = fill.size () + 1;

  if (dataSize != m_dataSize)
    {
      delete [] m_data;
      m_data = new uint8_t [dataSize];
      m_dataSize = dataSize;
    }

  memcpy (m_data, fill.c_str (), dataSize);

  //
  // Overwrite packet size attribute.
  //
  m_size = dataSize;
}

void
UdpDualSocket5gClient::SetFill (uint8_t fill, uint32_t dataSize)
{
  NS_LOG_FUNCTION (this << (uint) fill << dataSize);
  if (dataSize != m_dataSize)
    {
      delete [] m_data;
      m_data = new uint8_t [dataSize];
      m_dataSize = dataSize;
    }

  memset (m_data, fill, dataSize);

  //
  // Overwrite packet size attribute.
  //
  m_size = dataSize;
}

void
UdpDualSocket5gClient::SetFill (uint8_t *fill, uint32_t fillSize, uint32_t dataSize)
{
  NS_LOG_FUNCTION (this << fill << fillSize << dataSize);
  if (dataSize != m_dataSize)
    {
      delete [] m_data;
      m_data = new uint8_t [dataSize];
      m_dataSize = dataSize;
    }

  if (fillSize >= dataSize)
    {
      memcpy (m_data, fill, dataSize);
      m_size = dataSize;
      return;
    }

  //
  // Do all but the final fill.
  //
  uint32_t filled = 0;
  while (filled + fillSize < dataSize)
    {
      memcpy (&m_data[filled], fill, fillSize);
      filled += fillSize;
    }

  //
  // Last fill may be partial
  //
  memcpy (&m_data[filled], fill, dataSize - filled);

  //
  // Overwrite packet size attribute.
  //
  m_size = dataSize;
}

void
UdpDualSocket5gClient::UpdateNrSinr (Ptr<UdpDualSocket5gClient> dsClient, std::string path, RxPacketTraceParams params) {
  dsClient->SetNrSinrReceived (10*log10 (params.m_sinr)); //Set private attribute to the value of SINR received
}

void
UdpDualSocket5gClient::UpdateNrSinr2 (Ptr<UdpDualSocket5gClient> dsClient, std::string path, RxPacketTraceParams params) {
  dsClient->SetNrSinr2Received (10*log10 (params.m_sinr)); //Set private attribute to the value of SINR received
}

void
UdpDualSocket5gClient::SetNrSinrReceived (double sinr)
{
  NS_LOG_INFO ("SINR (dB) received from interface 1: " << sinr);
  m_nrSinrValueReceived = sinr;
  m_inputData.SINRDirect = sinr;
}

void
UdpDualSocket5gClient::SetNrSinr2Received (double sinr)
{
  NS_LOG_INFO ("SINR (dB) received from interface 2: " << sinr);
  m_nrSinrValueReceived2 = sinr;
  m_inputData.SINRIndirect = sinr;
}

void
UdpDualSocket5gClient::ScheduleTransmit (Time dt)
{
  NS_LOG_FUNCTION (this << dt);
  m_sendEvent = Simulator::Schedule (dt, &UdpDualSocket5gClient::Send, this);
}

void
UdpDualSocket5gClient::Send (void)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_sendEvent.IsExpired ());

  static uint64_t count = 0; //packet ID, starts with value 0
  Ptr<Packet> p;

	if (m_dataSize)
		{
			//
			// If m_dataSize is non-zero, we have a data buffer of the same size that we
			// are expected to copy and send.  This state of affairs is created if one of
			// the Fill functions is called.  In this case, m_size must have been set
			// to agree with m_dataSize
			//
			NS_ASSERT_MSG (m_dataSize == m_size, "UdpDualSocket5gClient::Send(): m_size and m_dataSize inconsistent");
			NS_ASSERT_MSG (m_data, "UdpDualSocket5gClient::Send(): m_dataSize but no m_data");
			p = Create<Packet> (m_data, m_dataSize);
		}
	else
		{
			//
			// If m_dataSize is zero, the client has indicated that it doesn't care
			// about the data itself either by specifying the data size by setting
			// the corresponding attribute or by not calling a SetFill function.  In
			// this case, we don't worry about it either.  But we do allow m_size
			// to have a value different from the (zero) m_dataSize.
			//
			p = Create<Packet> (m_size);
		}

  //std::cout << "SOck direct: " << localAddress1  << " Sock Indirect " << localAddress2 << std::endl;

  // Add packetTag (uniqueID and timestamp)
  UniquePacketIdTag packetTag;
  packetTag.SetSenderTimestamp (Simulator::Now ());
  packetTag.SetUniquePacketId (count);
  packetTag.SetSinrBeforeTxPacket (m_inputData.SINRDirect);
  packetTag.SetSinrIndirectBeforeTxPacket (m_inputData.SINRIndirect);
  p->AddByteTag (packetTag);
  m_txTrace (p);


//Suponemos direcciones IPv4, en caso de querer IPv6, habria que adaptar el codigo
//siguiendo el modelo de udp-echo-client.cc -> Send function
  if (connectionSelected == "BOTH") //send the packet via two interfaces (packet duplication)
    {
      LogTx (2);
      if (Ipv4Address::IsMatchingType (m_peerAddress))
        {
          m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
          m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
        }
      else if (Ipv6Address::IsMatchingType (m_peerAddress))
        {
          m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
          m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
        }

    //  p->RemoveAllByteTags (); //el paquete se envia sin el bytetag -> solo se utiliza para las trazas
      m_socketDirect->Send(p);
  	  m_socketIndirect->Send(p);
  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Direct connection)");

  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Indirect connection)");
    }
  else if (connectionSelected == "DIRECT") //Only send the packet via NR interface (direct connection)
    {
      LogTx(1);
      if (Ipv4Address::IsMatchingType (m_peerAddress))
        {
          m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
        }
      else if (Ipv6Address::IsMatchingType (m_peerAddress))
        {
          m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
        }

    //  p->RemoveAllByteTags (); //el paquete se envia sin el bytetag -> solo se utiliza para las trazas
      m_socketDirect->Send(p);
  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Direct connection)");
    }
  else { //connectionSelected == "INDIRECT", send the packet via NR indirect interface  (indirect connection)
    LogTx(3);
    if (Ipv4Address::IsMatchingType (m_peerAddress))
      {
        m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
      }
    else if (Ipv6Address::IsMatchingType (m_peerAddress))
      {
        m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort), m_inputData.SINRDirect);
      }

    //  p->RemoveAllByteTags (); //el paquete se envia sin el bytetag -> solo se utiliza para las trazas
    m_socketIndirect->Send(p);
    NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Indirect connection)");
  }


  ++m_sent;
  count++;

  //double minTimeBetweenPackets = 0.1; // In seconds

  if (m_sent < m_count)
    {
		//Opcional: AÃ±adir aqui tiempo intervalo exponencial con
	  //m_interval = Seconds(m_exponentialArrival->GetValue() + minTimeBetweenPackets);
      ScheduleTransmit (m_interval);
    }
}

void
UdpDualSocket5gClient::HandleReadDirect (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      LogRxDirect(packet);
      if (InetSocketAddress::IsMatchingType (from))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client received " << packet->GetSize () << " bytes from " << InetSocketAddress::ConvertFrom (from).GetIpv4 () << " port " << InetSocketAddress::ConvertFrom (from).GetPort () << " (Direct connection)");
        }
      else if (Inet6SocketAddress::IsMatchingType (from))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client received " << packet->GetSize () << " bytes from " << Inet6SocketAddress::ConvertFrom (from).GetIpv6 () << " port " << Inet6SocketAddress::ConvertFrom (from).GetPort () << " (Direct connection)");
        }
      m_rxTrace (packet);
      m_rxTraceWithAddresses (packet, from, m_netDeviceDirect->GetAddress());
    }
}

void
UdpDualSocket5gClient::HandleReadIndirect (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      LogRxIndirect(packet);
      if (InetSocketAddress::IsMatchingType (from))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client received " << packet->GetSize () << " bytes from " << InetSocketAddress::ConvertFrom (from).GetIpv4 () << " port " << InetSocketAddress::ConvertFrom (from).GetPort () << " (Indirect connection)");
        }
      else if (Inet6SocketAddress::IsMatchingType (from))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client received " << packet->GetSize () << " bytes from " << Inet6SocketAddress::ConvertFrom (from).GetIpv6 () << " port " << Inet6SocketAddress::ConvertFrom (from).GetPort () << " (Indirect connection)");
        }
      m_rxTrace (packet);
      m_rxTraceWithAddresses (packet, from, m_netDeviceIndirect->GetAddress());
    }
}

void
UdpDualSocket5gClient::LogTx (uint criteriaForTransmission)
{
	NS_LOG_FUNCTION(this << m_sent);

	switch(criteriaForTransmission)
	{
		case 1:
      m_packetRecord[m_sent].connection = "DIRECT";
			break;
		case 2:
      m_packetRecord[m_sent].connection = "BOTH";
			break;
    case 3:
      m_packetRecord[m_sent].connection = "INDIRECT";
		default:
			break;
	}

	m_packetRecord[m_sent].SINRDirect = m_inputData.SINRDirect;
	m_packetRecord[m_sent].SINRIndirect = m_inputData.SINRIndirect;
	m_packetRecord[m_sent].txTimestamp = Simulator::Now ();
}

void
UdpDualSocket5gClient::LogRxDirect (Ptr<Packet> p)
{
	NS_LOG_FUNCTION(this << m_sent);

  auto eit = m_packetRecord.find(m_sent-1);

  eit->second.rxTimestamp = Simulator::Now ();
  eit->second.e2eRtt = (eit->second.rxTimestamp - eit->second.txTimestamp).GetNanoSeconds()/1e6;

  NS_LOG_INFO("E2E RTT: " << eit->second.e2eRtt << " ");

  //============= Average RTT update =============

  uint numDirect = 0;

  double cumSumDirect = 0;

  uint windowSize4Average = 10;

  std::map<uint32_t, m_trxBufferElement > directConnections;

  for(std::map<uint32_t, m_trxBufferElement >::iterator it = m_packetRecord.begin(); it != m_packetRecord.end(); it++)
  {
    if (it->second.e2eRtt)
    {
      directConnections[it->first] = it->second;

    }
  }

  for(std::map<uint32_t, m_trxBufferElement >::reverse_iterator it = directConnections.rbegin(); it != directConnections.rend(); it++)
  {
    if(numDirect < windowSize4Average)
    {
      cumSumDirect += it->second.e2eRtt;
    }

    numDirect++;
  }

  if (numDirect == 0)
    m_inputData.averageDirectRtt = NaN;
  else
    m_inputData.averageDirectRtt = cumSumDirect/std::min(windowSize4Average,numDirect);

  eit->second.averageDirectRtt = m_inputData.averageDirectRtt;

  NS_LOG_INFO("Average DIRECT RTT: " << m_inputData.averageDirectRtt << " ms");

}

void
UdpDualSocket5gClient::LogRxIndirect (Ptr<Packet> p)
{
	NS_LOG_FUNCTION(this << m_sent);

  auto eit = m_packetRecord.find(m_sent-1);

  eit->second.rxTimestamp = Simulator::Now ();
  eit->second.e2eRtt = (eit->second.rxTimestamp - eit->second.txTimestamp).GetNanoSeconds()/1e6;

  NS_LOG_INFO("E2E RTT: " << eit->second.e2eRtt << " ");

  //============= Average RTT update =============

  uint numIndirect = 0;

  double cumSumIndirect = 0;

  uint windowSize4Average = 10;

  std::map<uint32_t, m_trxBufferElement > indirectConnections;

  for(std::map<uint32_t, m_trxBufferElement >::iterator it = m_packetRecord.begin(); it != m_packetRecord.end(); it++)
  {
    if (it->second.e2eRtt)
    {
      indirectConnections[it->first] = it->second;

    }
  }

  for(std::map<uint32_t, m_trxBufferElement >::reverse_iterator it = indirectConnections.rbegin(); it != indirectConnections.rend(); it++)
  {
    if(numIndirect < windowSize4Average)
    {
      cumSumIndirect += it->second.e2eRtt;
    }

    numIndirect++;
  }

  if (numIndirect == 0)
    m_inputData.averageIndirectRtt = NaN;
  else
    m_inputData.averageIndirectRtt = cumSumIndirect/std::min(windowSize4Average,numIndirect);

  eit->second.averageIndirectRtt = m_inputData.averageIndirectRtt;

  NS_LOG_INFO("Average INDIRECT RTT: " << m_inputData.averageIndirectRtt << " ms");

}

} // Namespace ns3
