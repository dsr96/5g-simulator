/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "udp-dualsocket-helper.h"
#include "ns3/udp-dualsocket-server.h"
#include "ns3/udp-dualsocket-client.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"
#include <queue>

namespace ns3 {

UdpDualSocketServerHelper::UdpDualSocketServerHelper (uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocketServer::GetTypeId ());
  SetAttribute ("Port", UintegerValue (port));
}

void
UdpDualSocketServerHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
UdpDualSocketServerHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocketServerHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocketServerHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
UdpDualSocketServerHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<UdpDualSocketServer> ();
  node->AddApplication (app);

  return app;
}

UdpDualSocketClientHelper::UdpDualSocketClientHelper (Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocketClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (address));
  SetAttribute ("RemotePort", UintegerValue (port));
}

UdpDualSocketClientHelper::UdpDualSocketClientHelper (Ipv4Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocketClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

UdpDualSocketClientHelper::UdpDualSocketClientHelper (Ipv6Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocketClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

void
UdpDualSocketClientHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

void
UdpDualSocketClientHelper::SetFill (Ptr<Application> app, std::string fill)
{
  app->GetObject<UdpDualSocketClient>()->SetFill (fill);
}

void
UdpDualSocketClientHelper::SetFill (Ptr<Application> app, uint8_t fill, uint32_t dataLength)
{
  app->GetObject<UdpDualSocketClient>()->SetFill (fill, dataLength);
}

void
UdpDualSocketClientHelper::SetFill (Ptr<Application> app, uint8_t *fill, uint32_t fillLength, uint32_t dataLength)
{
  app->GetObject<UdpDualSocketClient>()->SetFill (fill, fillLength, dataLength);
}

ApplicationContainer
UdpDualSocketClientHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocketClientHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocketClientHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
UdpDualSocketClientHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<UdpDualSocketClient> ();
  node->AddApplication (app);
  //Ptr<UdpDualSocketClient> uapp = app->GetObject<UdpDualSocketClient>();
  //uapp->StartEnergyLogger(); esto no lo vamos a utilizar en principio para packet duplication
  return app;
}

} // namespace ns3
