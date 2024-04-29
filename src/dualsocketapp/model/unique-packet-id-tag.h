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

#ifndef UNIQUE_PACKET_ID_TAG_H
#define UNIQUE_PACKET_ID_TAG_H

#include "ns3/packet.h"
#include "ns3/nstime.h"
#include <stdint.h>

namespace ns3 {

class Tag;

/**
 * Tag to calculate the per-PDU delay from eNb PDCP to UE PDCP
 */
 struct UniquePacketId
 {
   uint64_t senderTimestamp;
   uint64_t packetId;
   double sinr;
   double sinrIndirect;
 };

class UniquePacketIdTag : public Tag
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId  GetTypeId (void);
  virtual TypeId  GetInstanceTypeId (void) const;

  /**
   * Create an empty PDCP tag
   */
  UniquePacketIdTag ();
  /**
   * Create an PDCP tag with the given senderTimestamp
   * \param senderTimestamp the time stamp
   */
  UniquePacketIdTag (Time senderTimestamp);

  virtual void  Serialize (TagBuffer i) const;
  virtual void  Deserialize (TagBuffer i);
  virtual uint32_t  GetSerializedSize () const;
  virtual void Print (std::ostream &os) const;

  /**
   * Get the instant when the PDCP delivers the PDU to the MAC SAP provider
   * @return the sender timestamp
   */
  Time  GetSenderTimestamp (void) const;

  /**
   * Set the sender timestamp
   * @param senderTimestamp time stamp of the instant when the PDCP delivers the PDU to the MAC SAP provider
   */
  void  SetSenderTimestamp (Time senderTimestamp);

  /**
   * Get the Application packet identifier
   * @return the packet identifier
   */
  uint64_t GetUniquePacketId (void) const;

  /**
   * Set the packet identifier
   * @param packetUniqueId
   */
  void SetUniquePacketId (uint64_t uniquePacketId);

  /**
   * Get SINR value (dB) in UE before decide to transmit via 1 or 2 interfaces.
   * @return sinr value
   */
  double GetSinrBeforeTxPacket (void) const;

  /**
   * Set sinr value (dB) in UE before Tx the packet (sinr value to make packet duplication decision)
   * @param sinr
   */
  void SetSinrBeforeTxPacket (double sinr);

  /**
   * Get SINR value (dB) in UE before decide to transmit via 1 or 2 interfaces.
   * @return sinr value
   */
  double GetSinrIndirectBeforeTxPacket (void) const;

  /**
   * Set sinr value (dB) in UE before Tx the packet (sinr value to make packet duplication decision)
   * @param sinr
   */
  void SetSinrIndirectBeforeTxPacket (double sinr);

private:
  Time m_senderTimestamp; ///< sender timestamp
  UniquePacketId m_uniquePacketId;
};

} //namespace ns3

#endif /* PDCP_TAG_H */
