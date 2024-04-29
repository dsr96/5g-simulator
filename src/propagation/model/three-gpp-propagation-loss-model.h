/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
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

#ifndef THREE_GPP_PROPAGATION_LOSS_MODEL_H
#define THREE_GPP_PROPAGATION_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/buildings-module.h"

namespace ns3 {

/**
* \brief Base class for the 3GPP propagation models
*/
class ThreeGppPropagationLossModel : public PropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppPropagationLossModel () override;

  /**
   * \brief Set the channel condition model used to determine the channel
   *        state (e.g., the LOS/NLOS condition)
   * \param model pointer to the channel condition model
   */
  void SetChannelConditionModel (Ptr<ChannelConditionModel> model);

  /**
   * \brief Returns the associated channel condition model
   * \return the channel condition model
   */
  Ptr<ChannelConditionModel> GetChannelConditionModel (void) const;

  /**
   * \brief Set the central frequency of the model
   * \param f the central frequency in the range in Hz, between 500.0e6 and 100.0e9 Hz
   */
  void SetFrequency (double f);

  /**
   * \brief Return the current central frequency
   * \return The current central frequency
   */
  double GetFrequency (void) const;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppPropagationLossModel (const ThreeGppPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns the ThreeGppPropagationLossModel instance
   */
  ThreeGppPropagationLossModel & operator = (const ThreeGppPropagationLossModel &) = delete;

private:
  /**
   * Computes the received power by applying the pathloss model described in
   * 3GPP TR 38.901
   *
   * \param txPowerDbm tx power in dBm
   * \param a tx mobility model
   * \param b rx mobility model
   * \return the rx power in dBm
   */
  virtual double DoCalcRxPower (double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const override;

  /**
   * If this  model uses objects of type RandomVariableStream,
   * set the stream numbers to the integers starting with the offset
   * 'stream'. Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream
   * \return the number of stream indices assigned by this model
   */
  virtual int64_t DoAssignStreams (int64_t stream) override;

  /**
   * \brief Computes the pathloss between a and b
   * \param cond the channel condition
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLoss (Ptr<ChannelCondition> cond, double distance2D, double distance3D, double hUt, double hBs) const;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const = 0;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const = 0;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed by a vehicle. By default it raises an error to
   *        avoid misuse.
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossNlosv (double distance2D, double distance3D, double hUt, double hBs) const;

  /**
   * \brief Determines hUT and hBS. The default implementation assumes that
   *        the tallest node is the BS and the smallest is the UT. The derived classes
   * can change the default behavior by overriding this method.
   * \param za the height of the first node in meters
   * \param zb the height of the second node in meters
   * \return std::pair of heights in meters, the first element is hUt and the second is hBs
   */
  virtual std::pair<double, double> GetUtAndBsHeights (double za, double zb) const;

  /**
   * \brief Retrieves the shadowing value by looking at m_shadowingMap.
   *        If not found or if the channel condition changed it generates a new
   *        independent realization and stores it in the map, otherwise it correlates
   *        the new value with the previous one using the autocorrelation function
   *        defined in 3GPP TR 38.901, Sec. 7.4.4.
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing loss in dB
   */
  double GetShadowing (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const = 0;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const = 0;

  /**
   * \brief Returns an unique key for the channel between a and b.
   *
   * The key is the value of the Cantor function calculated by using as
   * first parameter the lowest node ID, and as a second parameter the highest
   * node ID.
   *
   * \param a tx mobility model
   * \param b rx mobility model
   * \return channel key
   */
  static uint32_t GetKey (Ptr<MobilityModel> a, Ptr<MobilityModel> b);

  /**
   * \brief Get the difference between the node position
   *
   * The difference is calculated as (b-a) if Id(a) < Id (b), or
   * (a-b) if Id(b) <= Id(a).
   *
   * \param a First node
   * \param b Second node
   * \return the difference between the node vector position
   */
  static Vector GetVectorDifference (Ptr<MobilityModel> a, Ptr<MobilityModel> b);

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const = 0;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const = 0;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const = 0;

protected:
  virtual void DoDispose () override;

  /**
  * \brief Computes the 2D distance between two 3D vectors
  * \param a the first 3D vector
  * \param b the second 3D vector
  * \return the 2D distance between a and b
  */
  static double Calculate2dDistance (Vector a, Vector b);

  //The IsLineIntersectBuildings method is based on
  //ISLineInBox method implemented in Bounding Box Types.
  //Link: http://www.3dkingdoms.com/weekly/weekly.php?a=21.
  int8_t IsLineIntersectBuildings (Vector L1, Vector L2 ) const;

  Ptr<ChannelConditionModel> m_channelConditionModel; //!< pointer to the channel condition model
  double m_frequency; //!< operating frequency in Hz
  bool m_shadowingEnabled; //!< enable/disable shadowing
  bool m_buildingsEnabled; //!< enable buildings losses in propagation model
  Ptr<NormalRandomVariable> m_normRandomVariable; //!< normal random variable

  /** Define a struct for the m_shadowingMap entries */
  struct ShadowingMapItem
  {
    double m_shadowing; //!< the shadowing loss in dB
    ChannelCondition::LosConditionValue m_condition; //!< the LOS/NLOS condition
    Vector m_distance; //!< the vector AB
  };

  mutable std::unordered_map<uint32_t, ShadowingMapItem> m_shadowingMap; //!< map to store the shadowing values
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901, Table 7.4.1-1
 *        for the RMa scenario.
 */
class ThreeGppRmaPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppRmaPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppRmaPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppRmaPropagationLossModel (const ThreeGppRmaPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns the ThreeGppRmaPropagationLossModel instance
   */
  ThreeGppRmaPropagationLossModel & operator = (const ThreeGppRmaPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Computes the PL1 formula for the RMa scenario
   * \param frequency the operating frequency in Hz
   * \param distance3D the 3D distance between the tx and the rx nodes in meters
   * \param h the average building height in meters
   * \param w the average street width in meters
   * \return result of the PL1 formula
   */
  static double Pl1 (double frequency, double distance3D, double h, double w);

  /**
   * \brief Computes the breakpoint distance for the RMa scenario
   * \param frequency the operating frequency in Hz
   * \param hA height of the tx node in meters
   * \param hB height of the rx node in meters
   * \return the breakpoint distance in meters
   */
  static double GetBpDistance (double frequency, double hA, double hB);

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;

  double m_h; //!< average building height in meters
  double m_w; //!< average street width in meters
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901, Table 7.4.1-1
 *        for the UMa scenario.
 */
class ThreeGppUmaPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppUmaPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppUmaPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppUmaPropagationLossModel (const ThreeGppUmaPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppUmaPropagationLossModel instance
   */
  ThreeGppUmaPropagationLossModel & operator = (const ThreeGppUmaPropagationLossModel &) = delete;

private:
  /**
   * If this  model uses objects of type RandomVariableStream,
   * set the stream numbers to the integers starting with the offset
   * 'stream'. Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream
   * \return the number of stream indices assigned by this model
   */
  virtual int64_t DoAssignStreams (int64_t stream) override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed.
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Computes the breakpoint distance
   * \param hUt height of the UT in meters
   * \param hBs height of the BS in meters
   * \param distance2D distance between the two nodes in meters
   * \return the breakpoint distance in meters
   */
  double GetBpDistance (double hUt, double hBs, double distance2D) const;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;

  Ptr<UniformRandomVariable> m_uniformVar; //!< a uniform random variable used for the computation of the breakpoint distance
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901, Table 7.4.1-1
 *        for the UMi-Street Canyon scenario.
 */
class ThreeGppUmiStreetCanyonPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppUmiStreetCanyonPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppUmiStreetCanyonPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppUmiStreetCanyonPropagationLossModel (const ThreeGppUmiStreetCanyonPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppUmiStreetCanyonPropagationLossModel instance
   */
  ThreeGppUmiStreetCanyonPropagationLossModel & operator = (const ThreeGppUmiStreetCanyonPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed.
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Computes the breakpoint distance
   * \param hUt height of the UT node in meters
   * \param hBs height of the BS node in meters
   * \param distance2D distance between the two nodes in meters
   * \return the breakpoint distance in meters
   */
  double GetBpDistance (double hUt, double hBs, double distance2D) const;

  /**
   * \brief Determines hUT and hBS. Overrides the default implementation.
   * \param za the height of the first node in meters
   * \param zb the height of the second node in meters
   * \return std::pair, the first element is hUt and the second is hBs
   */
  std::pair<double, double> GetUtAndBsHeights (double za, double zb) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;

};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901, Table 7.4.1-1
 *        for the Indoor Office scenario.
 */
class ThreeGppIndoorOfficePropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppIndoorOfficePropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppIndoorOfficePropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppIndoorOfficePropagationLossModel (const ThreeGppIndoorOfficePropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppIndoorOfficePropagationLossModel instance
   */
  ThreeGppIndoorOfficePropagationLossModel & operator = (const ThreeGppIndoorOfficePropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901 16.1.0, Table 7.4.1-1
 *        for the Indoor Factory SL.
 */
class ThreeGppIndoorFactorySLPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppIndoorFactorySLPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppIndoorFactorySLPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppIndoorFactorySLPropagationLossModel (const ThreeGppIndoorFactorySLPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppIndoorFactorySLPropagationLossModel instance
   */
  ThreeGppIndoorFactorySLPropagationLossModel & operator = (const ThreeGppIndoorFactorySLPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901 16.1.0, Table 7.4.1-1
 *        for the Indoor Factory DL.
 */
class ThreeGppIndoorFactoryDLPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppIndoorFactoryDLPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppIndoorFactoryDLPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppIndoorFactoryDLPropagationLossModel (const ThreeGppIndoorFactoryDLPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppIndoorFactoryDLPropagationLossModel instance
   */
  ThreeGppIndoorFactoryDLPropagationLossModel & operator = (const ThreeGppIndoorFactoryDLPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901 16.1.0, Table 7.4.1-1
 *        for the Indoor Factory SH.
 */
class ThreeGppIndoorFactorySHPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppIndoorFactorySHPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppIndoorFactorySHPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppIndoorFactorySHPropagationLossModel (const ThreeGppIndoorFactorySHPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppIndoorFactorySHPropagationLossModel instance
   */
  ThreeGppIndoorFactorySHPropagationLossModel & operator = (const ThreeGppIndoorFactorySHPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined in 3GPP TR 38.901 16.1.0, Table 7.4.1-1
 *        for the Indoor Factory DH.
 */
class ThreeGppIndoorFactoryDHPropagationLossModel : public ThreeGppPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  ThreeGppIndoorFactoryDHPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~ThreeGppIndoorFactoryDHPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ThreeGppIndoorFactoryDHPropagationLossModel (const ThreeGppIndoorFactoryDHPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \return the ThreeGppIndoorFactoryDHPropagationLossModel instance
   */
  ThreeGppIndoorFactoryDHPropagationLossModel & operator = (const ThreeGppIndoorFactoryDHPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;

  /**
  * Get O2I Building Penetrarion loss TABLE 7.4.3-2 3GPP 38.901 (PL_tw)
  * return loss PL_tw
  */
  virtual double GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /**
  * Get O2I Building Penetrarion standar deviation TABLE 7.4.3-2 3GPP 38.901 (standar deviation)
  * return standar deviation
  */
  virtual double GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const override;

  /*
  * Get Uniform Distance 2d IN (O2I)
  */
  virtual double GetDistance2dIn () const override;
};

} // namespace ns3

#endif /* THREE_GPP_PROPAGATION_LOSS_MODEL_H */
