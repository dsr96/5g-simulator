/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 CTTC
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
 * Author: Jaume Nin <jaume.nin@cttc.es>
 *         Nicola Baldo <nbaldo@cttc.es>
 */

#include "unique-packet-id-tag.h"
#include "ns3/tag.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (UniquePacketIdTag);

UniquePacketIdTag::UniquePacketIdTag ()
  : m_senderTimestamp (Seconds (0))
{
  // Nothing to do here
}


UniquePacketIdTag::UniquePacketIdTag (Time senderTimestamp)
  : m_senderTimestamp (senderTimestamp)

{
  // Nothing to do here
}

TypeId
UniquePacketIdTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UniquePacketIdTag")
    .SetParent<Tag> ()
    .SetGroupName("Applications")
    .AddConstructor<UniquePacketIdTag> ();
  return tid;
}

TypeId
UniquePacketIdTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
UniquePacketIdTag::GetSerializedSize (void) const
{
  return sizeof(m_uniquePacketId);
}

void
UniquePacketIdTag::Serialize (TagBuffer i) const
{
  //uint64_t senderTimestamp = m_senderTimestamp.GetNanoSeconds ();
  i.WriteU64 (m_uniquePacketId.senderTimestamp);
  i.WriteU64 (m_uniquePacketId.packetId);
  i.WriteDouble (m_uniquePacketId.sinr);
  i.WriteDouble  (m_uniquePacketId.sinrIndirect);
}

void
UniquePacketIdTag::Deserialize (TagBuffer i)
{
  m_uniquePacketId.senderTimestamp = i.ReadU64 ();
  m_uniquePacketId.packetId = i.ReadU64();
  m_uniquePacketId.sinr = i.ReadDouble();
  m_uniquePacketId.sinrIndirect = i.ReadDouble();
  m_senderTimestamp   = NanoSeconds (m_uniquePacketId.senderTimestamp);
}

void
UniquePacketIdTag::Print (std::ostream &os) const
{
  os << "Sender TIMESTAMP= " << NanoSeconds(m_uniquePacketId.senderTimestamp) << ", UniquePacketId= " << m_uniquePacketId.packetId << ", Sinr= " << m_uniquePacketId.sinr << ", sinrIndirect= " << m_uniquePacketId.sinrIndirect;
}

Time
UniquePacketIdTag::GetSenderTimestamp (void) const
{
  return NanoSeconds(m_uniquePacketId.senderTimestamp);
}

void
UniquePacketIdTag::SetSenderTimestamp (Time senderTimestamp)
{
  this->m_uniquePacketId.senderTimestamp = senderTimestamp.GetNanoSeconds();
  this->m_senderTimestamp = senderTimestamp;
}

uint64_t
UniquePacketIdTag::GetUniquePacketId (void) const
{
  return m_uniquePacketId.packetId;
}

void
UniquePacketIdTag::SetUniquePacketId (uint64_t uniquePacketId)
{
  this->m_uniquePacketId.packetId = uniquePacketId;
}

double
UniquePacketIdTag::GetSinrBeforeTxPacket (void) const
{
  return m_uniquePacketId.sinr;
}

void
UniquePacketIdTag::SetSinrBeforeTxPacket (double sinr)
{
  this->m_uniquePacketId.sinr = sinr;
}

double
UniquePacketIdTag::GetSinrIndirectBeforeTxPacket (void) const
{
  return m_uniquePacketId.sinrIndirect;
}

void
UniquePacketIdTag::SetSinrIndirectBeforeTxPacket (double sinr)
{
  this->m_uniquePacketId.sinrIndirect = sinr;
}

} // namespace ns3
