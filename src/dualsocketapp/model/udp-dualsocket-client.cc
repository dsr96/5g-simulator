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
#include "ns3/lte-module.h"
#include "ns3/callback.h"
#include "udp-dualsocket-client.h"
#include "unique-packet-id-tag.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UdpDualSocketClientApplication");

NS_OBJECT_ENSURE_REGISTERED (UdpDualSocketClient);

TypeId
UdpDualSocketClient::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UdpDualSocketClient")
    .SetParent<Application> ()
    .SetGroupName("Applications")
     .AddConstructor<UdpDualSocketClient> ()
    .AddAttribute ("MaxPackets",
                   "The maximum number of packets the application will send",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpDualSocketClient::m_count),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Interval",
                   "The time to wait between packets",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&UdpDualSocketClient::m_interval),
                   MakeTimeChecker ())
    .AddAttribute ("RemoteAddress",
                   "The destination Address of the outbound packets",
                   AddressValue (),
                   MakeAddressAccessor (&UdpDualSocketClient::m_peerAddress),
                   MakeAddressChecker ())
    .AddAttribute ("RemotePort",
                   "The destination port of the outbound packets",
                   UintegerValue (0),
                   MakeUintegerAccessor (&UdpDualSocketClient::m_peerPort),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketSize", "Size of echo data in outbound packets",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpDualSocketClient::SetDataSize,
                                         &UdpDualSocketClient::GetDataSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SinrThreshold", "SINR threshold value in (dB) for gNB. Below the value, turn on packet duplication",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&UdpDualSocketClient::m_sinrThreshold),
                   MakeDoubleChecker <double> ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&UdpDualSocketClient::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx", "A packet has been received",
                     MakeTraceSourceAccessor (&UdpDualSocketClient::m_rxTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("TxWithAddresses", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&UdpDualSocketClient::m_txTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
    .AddTraceSource ("RxWithAddresses", "A packet has been received",
                     MakeTraceSourceAccessor (&UdpDualSocketClient::m_rxTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
  ;
  return tid;
}

UdpDualSocketClient::UdpDualSocketClient ()
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
}

UdpDualSocketClient::~UdpDualSocketClient()
{
  NS_LOG_FUNCTION (this);
  m_socketDirect = 0;
  m_socketIndirect = 0;

  delete [] m_data;
  m_data = 0;
  m_dataSize = 0;
}

void
UdpDualSocketClient::SetRemote (Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = ip;
  m_peerPort = port;
}

void
UdpDualSocketClient::SetRemote (Ipv4Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = Address (ip);
  m_peerPort = port;
}

void
UdpDualSocketClient::SetRemote (Ipv6Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = Address (ip);
  m_peerPort = port;
}

void
UdpDualSocketClient::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
UdpDualSocketClient::StartApplication (void)
{
	NS_LOG_FUNCTION (this);

	Ptr<Node> node = GetNode();

//	for (uint k = 0; k < node->GetNDevices(); k++){

		Ptr<NrUeNetDevice> dev1 = node->GetDevice(0)->GetObject<NrUeNetDevice>();
		Ptr<LteUeNetDevice> dev2 = node->GetDevice(1)->GetObject<LteUeNetDevice>();

		if (dev1)
		m_netDeviceDirect = node->GetDevice(0)->GetObject<NrUeNetDevice>();

		if (dev2)
		m_netDeviceIndirect = node->GetDevice(1)->GetObject<LteUeNetDevice>();
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

		m_socketDirect->SetRecvCallback (MakeCallback (&UdpDualSocketClient::HandleReadDirect, this));
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

		m_socketIndirect->SetRecvCallback (MakeCallback (&UdpDualSocketClient::HandleReadIndirect, this));
	}

	ScheduleTransmit (Seconds (0.));
}

void
UdpDualSocketClient::StopApplication ()
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
UdpDualSocketClient::SetDataSize (uint32_t dataSize)
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
UdpDualSocketClient::GetDataSize (void) const
{
  NS_LOG_FUNCTION (this);
  return m_size;
}

void
UdpDualSocketClient::SetFill (std::string fill)
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
UdpDualSocketClient::SetFill (uint8_t fill, uint32_t dataSize)
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
UdpDualSocketClient::SetFill (uint8_t *fill, uint32_t fillSize, uint32_t dataSize)
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
UdpDualSocketClient::UpdateGnbNrSinr (Ptr<UdpDualSocketClient> dsClient, std::string path, RxPacketTraceParams params) {
  dsClient->SetGnbNrSinrReceived (10*log10 (params.m_sinr)); //Set private attribute to the value of SINR received
}

void
UdpDualSocketClient::SetGnbNrSinrReceived (double sinr)
{
  NS_LOG_INFO ("SINR (dB) received at gNB: " << sinr);
  m_nrSinrValueReceived = sinr;
}

void
UdpDualSocketClient::ScheduleTransmit (Time dt)
{
  NS_LOG_FUNCTION (this << dt);
  m_sendEvent = Simulator::Schedule (dt, &UdpDualSocketClient::Send, this);
}

void
UdpDualSocketClient::Send (void)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_sendEvent.IsExpired ());

  static uint64_t count = 0; //packet ID, starts with value 0
  Ptr<Packet> p;
  double currentSinr;

	if (m_dataSize)
		{
			//
			// If m_dataSize is non-zero, we have a data buffer of the same size that we
			// are expected to copy and send.  This state of affairs is created if one of
			// the Fill functions is called.  In this case, m_size must have been set
			// to agree with m_dataSize
			//
			NS_ASSERT_MSG (m_dataSize == m_size, "UdpDualSocketClient::Send(): m_size and m_dataSize inconsistent");
			NS_ASSERT_MSG (m_data, "UdpDualSocketClient::Send(): m_dataSize but no m_data");
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

  // Decide what socket to send the message through
    // call to the trace sinks before the packet is actually sent,
    // so that tags added to the packet can be sent as well
  currentSinr = m_nrSinrValueReceived;
  // Add packetTag (uniqueID and timestamp)
  UniquePacketIdTag packetTag;
  packetTag.SetSenderTimestamp (Simulator::Now ());
  packetTag.SetUniquePacketId (count);
  packetTag.SetSinrBeforeTxPacket (currentSinr);
  p->AddByteTag (packetTag);
  m_txTrace (p);

  if (Ipv4Address::IsMatchingType (m_peerAddress))
    {
      m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort));
      m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), InetSocketAddress (Ipv4Address::ConvertFrom (m_peerAddress), m_peerPort));
    }
  else if (Ipv6Address::IsMatchingType (m_peerAddress))
    {
      m_txTraceWithAddresses (p, m_netDeviceDirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort));
      m_txTraceWithAddresses (p, m_netDeviceIndirect->GetAddress(), Inet6SocketAddress (Ipv6Address::ConvertFrom (m_peerAddress), m_peerPort));
    }

//Suponemos direcciones IPv4, en caso de querer IPv6, habria que adaptar el codigo
//siguiendo el modelo de udp-echo-client.cc -> Send function
  if ((currentSinr < m_sinrThreshold) || (currentSinr < 0.0)) //send the packet via two interfaces (packet duplication)
    {
      m_socketDirect->Send(p);
  	  m_socketIndirect->Send(p);
  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Direct connection)");

  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Indirect connection)");
    }
  else //Only send the packet via NR interface (direct connection)
    {
      m_socketDirect->Send(p);
  		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << "s client sent " << m_size << " bytes to " << Ipv4Address::ConvertFrom (m_peerAddress) << " port " << m_peerPort << " (Direct connection)");
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
UdpDualSocketClient::HandleReadDirect (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      //LogRx(packet);
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
UdpDualSocketClient::HandleReadIndirect (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      //LogRx(packet);
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

} // Namespace ns3
