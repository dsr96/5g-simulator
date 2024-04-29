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
#include "udp-dualsocket5g-helper.h"
#include "ns3/udp-dualsocket5g-server.h"
#include "ns3/udp-dualsocket5g-client.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"
#include <queue>

namespace ns3 {

UdpDualSocket5gServerHelper::UdpDualSocket5gServerHelper (uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocket5gServer::GetTypeId ());
  SetAttribute ("Port", UintegerValue (port));
}

void
UdpDualSocket5gServerHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
UdpDualSocket5gServerHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocket5gServerHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocket5gServerHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
UdpDualSocket5gServerHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<UdpDualSocket5gServer> ();
  node->AddApplication (app);

  return app;
}

UdpDualSocket5gClientHelper::UdpDualSocket5gClientHelper (Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocket5gClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (address));
  SetAttribute ("RemotePort", UintegerValue (port));
}

UdpDualSocket5gClientHelper::UdpDualSocket5gClientHelper (Ipv4Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocket5gClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

UdpDualSocket5gClientHelper::UdpDualSocket5gClientHelper (Ipv6Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpDualSocket5gClient::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

void
UdpDualSocket5gClientHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

void
UdpDualSocket5gClientHelper::SetFill (Ptr<Application> app, std::string fill)
{
  app->GetObject<UdpDualSocket5gClient>()->SetFill (fill);
}

void
UdpDualSocket5gClientHelper::SetFill (Ptr<Application> app, uint8_t fill, uint32_t dataLength)
{
  app->GetObject<UdpDualSocket5gClient>()->SetFill (fill, dataLength);
}

void
UdpDualSocket5gClientHelper::SetFill (Ptr<Application> app, uint8_t *fill, uint32_t fillLength, uint32_t dataLength)
{
  app->GetObject<UdpDualSocket5gClient>()->SetFill (fill, fillLength, dataLength);
}

ApplicationContainer
UdpDualSocket5gClientHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocket5gClientHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
UdpDualSocket5gClientHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
UdpDualSocket5gClientHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<UdpDualSocket5gClient> ();
  node->AddApplication (app);
  //Ptr<UdpDualSocket5gClient> uapp = app->GetObject<UdpDualSocket5gClient>();
  //uapp->StartEnergyLogger(); esto no lo vamos a utilizar en principio para packet duplication
  return app;
}

} // namespace ns3
