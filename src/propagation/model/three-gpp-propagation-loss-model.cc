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

#include "three-gpp-propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include <cmath>
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/buildings-module.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ThreeGppPropagationLossModel");

static const double M_C = 3.0e8; // propagation velocity in free space

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppPropagationLossModel);

TypeId
ThreeGppPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddAttribute ("Frequency", "The centre frequency in Hz.",
                   DoubleValue (500.0e6),
                   MakeDoubleAccessor (&ThreeGppPropagationLossModel::SetFrequency,
                                       &ThreeGppPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ShadowingEnabled", "Enable/disable shadowing.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ThreeGppPropagationLossModel::m_shadowingEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("ChannelConditionModel", "Pointer to the channel condition model.",
                   PointerValue (),
                   MakePointerAccessor (&ThreeGppPropagationLossModel::SetChannelConditionModel,
                                        &ThreeGppPropagationLossModel::GetChannelConditionModel),
                   MakePointerChecker<ChannelConditionModel> ())
    .AddAttribute ("BuildingsEnabled", "Enable/disable buildings.",
                    BooleanValue (false),
                    MakeBooleanAccessor (&ThreeGppPropagationLossModel::m_buildingsEnabled),
                    MakeBooleanChecker ())
  ;
  return tid;
}

ThreeGppPropagationLossModel::ThreeGppPropagationLossModel ()
  : PropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // initialize the normal random variable
  m_normRandomVariable = CreateObject<NormalRandomVariable> ();
  m_normRandomVariable->SetAttribute ("Mean", DoubleValue (0));
  m_normRandomVariable->SetAttribute ("Variance", DoubleValue (1));
}

ThreeGppPropagationLossModel::~ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
ThreeGppPropagationLossModel::DoDispose ()
{
  m_channelConditionModel->Dispose ();
  m_channelConditionModel = nullptr;
  m_shadowingMap.clear ();
}

void
ThreeGppPropagationLossModel::SetChannelConditionModel (Ptr<ChannelConditionModel> model)
{
  NS_LOG_FUNCTION (this);
  m_channelConditionModel = model;
}

Ptr<ChannelConditionModel>
ThreeGppPropagationLossModel::GetChannelConditionModel () const
{
  NS_LOG_FUNCTION (this);
  return m_channelConditionModel;
}

void
ThreeGppPropagationLossModel::SetFrequency (double f)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (f >= 500.0e6 && f <= 100.0e9, "Frequency should be between 0.5 and 100 GHz but is " << f);
  m_frequency = f;
}

double
ThreeGppPropagationLossModel::GetFrequency () const
{
  NS_LOG_FUNCTION (this);
  return m_frequency;
}

double
ThreeGppPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                             Ptr<MobilityModel> a,
                                             Ptr<MobilityModel> b) const
{
  NS_LOG_FUNCTION (this);

  // check if the model is initialized
  NS_ASSERT_MSG (m_frequency != 0.0, "First set the centre frequency");

  // retrieve the channel condition
  NS_ASSERT_MSG (m_channelConditionModel, "First set the channel condition model");
  Ptr<ChannelCondition> cond = m_channelConditionModel->GetChannelCondition (a, b);

  // compute the 2D distance between a and b
  double distance2d = Calculate2dDistance (a->GetPosition (), b->GetPosition ());

  // compute the 3D distance between a and b
  double distance3d = CalculateDistance (a->GetPosition (), b->GetPosition ());

  // compute hUT and hBS
  std::pair<double, double> heights = GetUtAndBsHeights (a->GetPosition ().z, b->GetPosition ().z);

  double rxPow = txPowerDbm;
  if (a == b) //Se trata de un UE con misma posicion- DC, asumimos pérdidas 0 y shadowing 0- no hay interferencia
  {
    return rxPow;
  }

  double lossIndoor = 0;

  if (m_buildingsEnabled == true)
  {
    //Comprobar si se intersecta con un edificio
    Ptr<MobilityBuildingInfo> a1 = a->GetObject<MobilityBuildingInfo> ();
    Ptr<MobilityBuildingInfo> b1 = b->GetObject<MobilityBuildingInfo> ();
    NS_ASSERT_MSG ((a1 != 0) && (b1 != 0), "Buildings only works with MobilityBuildingInfo");

    /* The IsOutdoor and IsIndoor function is only based on the initial node position,
    * it is not updated when the node is entering the building from outside or vise versa.*/
    if (!a1->IsIndoor () && !b1->IsIndoor ())
      {
        /* The outdoor case, determine LOS/NLOS
         * The channel condition should be NLOS if the line intersect one of the buildings, otherwise LOS. (esto en industria es diferente, puede haber NLOS aun sin intersectar)
         * */
        int8_t intersect = IsLineIntersectBuildings (a->GetPosition (), b->GetPosition ());
        if (intersect != 0)
          {
            //Si intersecta, probabilidad NLOS 100%
            cond->SetLosCondition (ChannelCondition::LosConditionValue::NLOS);
          }
        else
          {
            //Aqui en lugar de LOS, vamos a mantener la condición que tuviera del canal (la escoge mediante probabilidad, puede ser NLOS o LOS)
            //cond->SetLosCondition (ChannelCondition::LosConditionValue::NLOS);
            NS_LOG_INFO ("Se escoge la condición actual del canal");
          }

      }
    else if (a1->IsIndoor () && b1->IsIndoor ())
      {
        // Aqui se llega si hay UEs en el edificio y se está calculando la interferencia (mobility a y b son UEs, no se aplican las pérdidas O2I, sólo GNB-UE o viceversa)
        NS_LOG_INFO ("indoor propagation loss not implemented yet");
      }
    else           //outdoor to indoor case
      {
        //PL = PL_b + PL_tw + PL_in + N(0,sig^2); (7.4-2)
        //Here we assume the indoor nodes are all NLOS O2I.
        NS_LOG_INFO ("Outdoor to indoor case");

        cond->SetLosCondition (ChannelCondition::LosConditionValue::NLOS);
        //Compute the addition indoor pathloss term when this is the first transmission, or the node moves from outdoor to indoor.
        double PL_tw;
        double stdP;

        PL_tw = GetPlTw (a1, b1);
        stdP = GetO2IStdDeviation (a1, b1);
        lossIndoor += PL_tw;

        //compute PL_in
        Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
        Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();
        double dis_2D_in = GetDistance2dIn ();

        lossIndoor += 0.5 * dis_2D_in;
        //compute indoor shadowing
        Ptr<NormalRandomVariable> norRv = CreateObject<NormalRandomVariable> ();
        lossIndoor += stdP * norRv->GetValue ();
    }
  }

  rxPow -= GetLoss (cond, distance2d, distance3d, heights.first, heights.second);

  if (m_buildingsEnabled && lossIndoor != 0)
    {
      rxPow -= lossIndoor; //En caso de perdidas por building, no se le añade el shadowing.
    }

  if (m_shadowingEnabled && lossIndoor == 0)
    {
      rxPow -= GetShadowing (a, b, cond->GetLosCondition ());
    }

  return rxPow;
}

double
ThreeGppPropagationLossModel::GetLoss (Ptr<ChannelCondition> cond, double distance2d, double distance3d, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  double loss = 0;
  if (cond->GetLosCondition () == ChannelCondition::LosConditionValue::LOS)
    {
      loss = GetLossLos (distance2d, distance3d, hUt, hBs);
    }
  else if (cond->GetLosCondition () == ChannelCondition::LosConditionValue::NLOSv)
    {
      loss = GetLossNlosv (distance2d, distance3d, hUt, hBs);
    }
  else if (cond->GetLosCondition () == ChannelCondition::LosConditionValue::NLOS)
    {
      loss = GetLossNlos (distance2d, distance3d, hUt, hBs);
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }
  return loss;
}

double
ThreeGppPropagationLossModel::GetLossNlosv (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("Unsupported channel condition (NLOSv)");
  return 0;
}

double
ThreeGppPropagationLossModel::GetShadowing (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  double shadowingValue;

  // compute the channel key
  uint32_t key = GetKey (a, b);

  bool notFound = false; // indicates if the shadowing value has not been computed yet
  bool newCondition = false; // indicates if the channel condition has changed
  Vector newDistance; // the distance vector, that is not a distance but a difference
  auto it = m_shadowingMap.end (); // the shadowing map iterator
  if (m_shadowingMap.find (key) != m_shadowingMap.end ())
    {
      // found the shadowing value in the map
      it = m_shadowingMap.find (key);
      newDistance = GetVectorDifference (a, b);
      newCondition = (it->second.m_condition != cond); // true if the condition changed
    }
  else
    {
      notFound = true;

      // add a new entry in the map and update the iterator
      ShadowingMapItem newItem;
      it = m_shadowingMap.insert (it, std::make_pair (key, newItem));
    }

  if (notFound || newCondition)
    {
      // generate a new independent realization
      shadowingValue = m_normRandomVariable->GetValue () * GetShadowingStd (a, b, cond);
    }
  else
    {
      // compute a new correlated shadowing loss
      Vector2D displacement (newDistance.x - it->second.m_distance.x, newDistance.y - it->second.m_distance.y);
      double R = exp (-1 * displacement.GetLength () / GetShadowingCorrelationDistance (cond));
      shadowingValue =  R * it->second.m_shadowing + sqrt (1 - R * R) * m_normRandomVariable->GetValue () * GetShadowingStd (a, b, cond);
    }

  // update the entry in the map
  it->second.m_shadowing = shadowingValue;
  it->second.m_distance = newDistance; // Save the (0,0,0) vector in case it's the first time we are calculating this value
  it->second.m_condition = cond;

  return shadowingValue;
}

std::pair<double, double>
ThreeGppPropagationLossModel::GetUtAndBsHeights (double za, double zb) const
{
  // The default implementation assumes that the tallest node is the BS and the
  // smallest is the UT.
  double hUt = std::min (za, zb);
  double hBs = std::max (za, zb);

  return std::pair<double, double> (hUt, hBs);
}

int64_t
ThreeGppPropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this);

  m_normRandomVariable->SetStream (stream);
  return 1;
}

double
ThreeGppPropagationLossModel::Calculate2dDistance (Vector a, Vector b)
{
  double x = a.x - b.x;
  double y = a.y - b.y;
  double distance2D = sqrt (x * x + y * y);

  return distance2D;
}

uint32_t
ThreeGppPropagationLossModel::GetKey (Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
  // use the nodes ids to obtain an unique key for the channel between a and b
  // sort the nodes ids so that the key is reciprocal
  uint32_t x1 = std::min (a->GetObject<Node> ()->GetId (), b->GetObject<Node> ()->GetId ());
  uint32_t x2 = std::max (a->GetObject<Node> ()->GetId (), b->GetObject<Node> ()->GetId ());

  // use the cantor function to obtain the key
  uint32_t key = (((x1 + x2) * (x1 + x2 + 1)) / 2) + x2;

  return key;
}

Vector
ThreeGppPropagationLossModel::GetVectorDifference (Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
  uint32_t x1 = a->GetObject<Node> ()->GetId ();
  uint32_t x2 = b->GetObject<Node> ()->GetId ();

  if (x1 < x2)
    {
      return b->GetPosition () - a->GetPosition ();
    }
  else
    {
      return a->GetPosition () - b->GetPosition ();
    }
}

int8_t
ThreeGppPropagationLossModel::IsLineIntersectBuildings (Vector L1, Vector L2 ) const
{
  int8_t numIntersect = 0;
  for (BuildingList::Iterator bit = BuildingList::Begin (); bit != BuildingList::End (); ++bit)
    {
      Box boundaries = (*bit)->GetBoundaries ();
      /*std::cout << "X: " << 0.5 * (boundaries.xMax - boundaries.xMin) << " Y: " <<
                      0.5 * (boundaries.yMax - boundaries.yMin) << " Z: " <<
                      0.5 * (boundaries.zMax - boundaries.zMin) << std::endl;*/


      Vector boxSize (0.5 * (boundaries.xMax - boundaries.xMin),
                      0.5 * (boundaries.yMax - boundaries.yMin),
                      0.5 * (boundaries.zMax - boundaries.zMin));
      Vector boxCenter (boundaries.xMin + boxSize.x,
                        boundaries.yMin + boxSize.y,
                        boundaries.zMin + boxSize.z);
                      /*  std::cout << "X: " << (boundaries.xMin + boxSize.x) << " Y: "
                                          (boundaries.yMin + boxSize.y) << " Z:"
                                          (boundaries.zMin + boxSize.z) << std::endl; */
      // Put line in box space
      Vector LB1 (L1.x - boxCenter.x, L1.y - boxCenter.y, L1.z - boxCenter.z);
      Vector LB2 (L2.x - boxCenter.x, L2.y - boxCenter.y, L2.z - boxCenter.z);

      // Get line midpoint and extent
      Vector LMid (0.5 * (LB1.x + LB2.x), 0.5 * (LB1.y + LB2.y), 0.5 * (LB1.z + LB2.z));
      Vector L (LB1.x - LMid.x, LB1.y - LMid.y, LB1.z - LMid.z);
      Vector LExt ( std::abs (L.x), std::abs (L.y), std::abs (L.z) );

      // Use Separating Axis Test
      // Separation vector from box center to line center is LMid, since the line is in box space
      // If the line did not intersect this building, jump to the next building.
      if ( std::abs ( LMid.x ) > boxSize.x + LExt.x )
        {
          continue;
        }
      if ( std::abs ( LMid.y ) > boxSize.y + LExt.y )
        {
          continue;
        }
      if ( std::abs ( LMid.z ) > boxSize.z + LExt.z )
        {
          continue;
        }
      // Crossproducts of line and each axis
      if ( std::abs ( LMid.y * L.z - LMid.z * L.y)  >  (boxSize.y * LExt.z + boxSize.z * LExt.y) )
        {
          continue;
        }
      if ( std::abs ( LMid.x * L.z - LMid.z * L.x)  >  (boxSize.x * LExt.z + boxSize.z * LExt.x) )
        {
          continue;
        }
      if ( std::abs ( LMid.x * L.y - LMid.y * L.x)  >  (boxSize.x * LExt.y + boxSize.y * LExt.x) )
        {
          continue;
        }

      // No separating axis, the line intersects
      // If the line intersect this building, increase the counter.
      numIntersect++;
    }
  return numIntersect;
}
// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppRmaPropagationLossModel);

TypeId
ThreeGppRmaPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppRmaPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppRmaPropagationLossModel> ()
    .AddAttribute ("AvgBuildingHeight", "The average building height in meters.",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&ThreeGppRmaPropagationLossModel::m_h),
                   MakeDoubleChecker<double> (5.0, 50.0))
    .AddAttribute ("AvgStreetWidth", "The average street width in meters.",
                   DoubleValue (20.0),
                   MakeDoubleAccessor (&ThreeGppRmaPropagationLossModel::m_w),
                   MakeDoubleChecker<double> (5.0, 50.0))
  ;
  return tid;
}

ThreeGppRmaPropagationLossModel::ThreeGppRmaPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppRmaChannelConditionModel> ();
}

ThreeGppRmaPropagationLossModel::~ThreeGppRmaPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppRmaPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_frequency <= 30.0e9, "RMa scenario is valid for frequencies between 0.5 and 30 GHz.");

  // check if hBS and hUT are within the specified validity range
  if (hUt < 1.0 || hUt > 10.0)
    {
      NS_LOG_WARN ("The height of the UT should be between 1 and 10 m (see TR 38.901, Table 7.4.1-1)");
    }

  if (hBs < 10.0 || hBs > 150.0)
    {
      NS_LOG_WARN ("The height of the BS should be between 10 and 150 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  double distanceBp = GetBpDistance (m_frequency, hBs, hUt);
  NS_LOG_DEBUG ("breakpoint distance " << distanceBp);

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 10.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 0;
  if (distance2D <= distanceBp)
    {
      // use PL1
      loss = Pl1 (m_frequency, distance3D, m_h, m_w);
    }
  else
    {
      // use PL2
      loss = Pl1 (m_frequency, distanceBp, m_h, m_w) + 40 * log10 (distance3D / distanceBp);
    }

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppRmaPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_frequency <= 30.0e9, "RMa scenario is valid for frequencies between 0.5 and 30 GHz.");

  // check if hBs and hUt are within the validity range
  if (hUt < 1.0 || hUt > 10.0)
    {
      NS_LOG_WARN ("The height of the UT should be between 1 and 10 m (see TR 38.901, Table 7.4.1-1)");
    }

  if (hBs < 10.0 || hBs > 150.0)
    {
      NS_LOG_WARN ("The height of the BS should be between 10 and 150 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 5.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 161.04 - 7.1 * log10 (m_w) + 7.5 * log10 (m_h) - (24.37 - 3.7 * pow ((m_h / hBs), 2)) * log10 (hBs) + (43.42 - 3.1 * log10 (hBs)) * (log10 (distance3D) - 3.0) + 20.0 * log10 (m_frequency / 1e9) - (3.2 * pow (log10 (11.75 * hUt), 2) - 4.97);

  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppRmaPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      // compute the 2D distance between the two nodes
      double distance2d = Calculate2dDistance (a->GetPosition (), b->GetPosition ());

      // compute the breakpoint distance (see 3GPP TR 38.901, Table 7.4.1-1, note 5)
      double distanceBp = GetBpDistance (m_frequency, a->GetPosition ().z, b->GetPosition ().z);

      if (distance2d <= distanceBp)
        {
          shadowingStd = 4.0;
        }
      else
        {
          shadowingStd = 6.0;
        }
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 8.0;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppRmaPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double correlationDistance;

  // See 3GPP TR 38.901, Table 7.5-6
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 37;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 120;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppRmaPropagationLossModel::Pl1 (double frequency, double distance3D, double h, double w)
{
  NS_UNUSED (w);
  double loss = 20.0 * log10 (40.0 * M_PI * distance3D * frequency / 1e9 / 3.0) + std::min (0.03 * pow (h, 1.72), 10.0) * log10 (distance3D) - std::min (0.044 * pow (h, 1.72), 14.77) + 0.002 * log10 (h) * distance3D;
  return loss;
}

double
ThreeGppRmaPropagationLossModel::GetBpDistance (double frequency, double hA, double hB)
{
  double distanceBp = 2.0 * M_PI * hA * hB * frequency / M_C;
  return distanceBp;
}

double
ThreeGppRmaPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);
  // only low-loss model is applied to RMa
  double PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
  return PL_tw;
}

double
ThreeGppRmaPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);
  return 4.4;
}


double
ThreeGppRmaPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,10), uniRv2->GetValue (0,10)));
}
// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppUmaPropagationLossModel);

TypeId
ThreeGppUmaPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppUmaPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppUmaPropagationLossModel> ()
  ;
  return tid;
}

ThreeGppUmaPropagationLossModel::ThreeGppUmaPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
  m_uniformVar = CreateObject<UniformRandomVariable> ();

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppUmaChannelConditionModel> ();
}

ThreeGppUmaPropagationLossModel::~ThreeGppUmaPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppUmaPropagationLossModel::GetBpDistance (double hUt, double hBs, double distance2D) const
{
  NS_LOG_FUNCTION (this);

  // compute g (d2D) (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double g = 0.0;
  if (distance2D > 18.0)
    {
      g = 5.0 / 4.0 * pow (distance2D / 100.0, 3) * exp (-distance2D / 150.0);
    }

  // compute C (hUt, d2D) (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double c = 0.0;
  if (hUt >= 13.0)
    {
      c = pow ((hUt - 13.0) / 10.0, 1.5) * g;
    }

  // compute hE (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double prob = 1.0 / (1.0 + c);
  double hE = 0.0;
  if (m_uniformVar->GetValue () < prob)
    {
      hE = 1.0;
    }
  else
    {
      int random = m_uniformVar->GetInteger (12, (int)(hUt - 1.5));
      hE = (double)floor (random / 3.0) * 3.0;
    }

  // compute dBP' (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double distanceBp = 4 * (hBs - hE) * (hUt - hE) * m_frequency / M_C;

  return distanceBp;
}

double
ThreeGppUmaPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if hBS and hUT are within the validity range
  if (hUt < 1.5 || hUt > 22.5)
    {
      NS_LOG_WARN ("The height of the UT should be between 1.5 and 22.5 m (see TR 38.901, Table 7.4.1-1)");
    }

  if (hBs != 25.0)
    {
      NS_LOG_WARN ("The height of the BS should be equal to 25 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  // compute the breakpoint distance (see 3GPP TR 38.901, Table 7.4.1-1, note 1)
  double distanceBp = GetBpDistance (hUt, hBs, distance2D);
  NS_LOG_DEBUG ("breakpoint distance " << distanceBp);

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 5.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 0;
  if (distance2D <= distanceBp)
    {
      // use PL1
      loss = 28.0 + 22.0 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9);
    }
  else
    {
      // use PL2
      loss = 28.0 + 40.0 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9) - 9.0 * log10 (pow (distanceBp, 2) + pow (hBs - hUt, 2));
    }

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppUmaPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if hBS and hUT are within the vaalidity range
  if (hUt < 1.5 || hUt > 22.5)
    {
      NS_LOG_WARN ("The height of the UT should be between 1.5 and 22.5 m (see TR 38.901, Table 7.4.1-1)");
    }

  if (hBs != 25.0)
    {
      NS_LOG_WARN ("The height of the BS should be equal to 25 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 5.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 13.54 + 39.08 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9) - 0.6 * (hUt - 1.5);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);
  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppUmaPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 6.0;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppUmaPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double correlationDistance;

  // See 3GPP TR 38.901, Table 7.5-6
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 37;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 50;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

int64_t
ThreeGppUmaPropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this);

  m_normRandomVariable->SetStream (stream);
  m_uniformVar->SetStream (stream);
  return 2;
}

double
ThreeGppUmaPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  double PL_tw = 0;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return PL_tw;
}

double
ThreeGppUmaPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  double stdP;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return stdP;
}

double
ThreeGppUmaPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppUmiStreetCanyonPropagationLossModel);

TypeId
ThreeGppUmiStreetCanyonPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppUmiStreetCanyonPropagationLossModel> ()
  ;
  return tid;
}
ThreeGppUmiStreetCanyonPropagationLossModel::ThreeGppUmiStreetCanyonPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppUmiStreetCanyonChannelConditionModel> ();
}

ThreeGppUmiStreetCanyonPropagationLossModel::~ThreeGppUmiStreetCanyonPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetBpDistance (double hUt, double hBs, double distance2D) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);

  // compute hE (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double hE = 1.0;

  // compute dBP' (see 3GPP TR 38.901, Table 7.4.1-1, Note 1)
  double distanceBp = 4 * (hBs - hE) * (hUt - hE) * m_frequency / M_C;

  return distanceBp;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if hBS and hUT are within the validity range
  if (hUt < 1.5 || hUt >= 10.0)
    {
      NS_LOG_WARN ("The height of the UT should be between 1.5 and 22.5 m (see TR 38.901, Table 7.4.1-1). We further assume hUT < hBS, then hUT is upper bounded by hBS, which should be 10 m");
    }

  if (hBs != 10.0)
    {
      NS_LOG_WARN ("The height of the BS should be equal to 10 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  // compute the breakpoint distance (see 3GPP TR 38.901, Table 7.4.1-1, note 1)
  double distanceBp = GetBpDistance (hUt, hBs, distance2D);
  NS_LOG_DEBUG ("breakpoint distance " << distanceBp);

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 5.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 0;
  if (distance2D <= distanceBp)
    {
      // use PL1
      loss = 32.4 + 21.0 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9);
    }
  else
    {
      // use PL2
      loss = 32.4 + 40.0 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9) - 9.5 * log10 (pow (distanceBp, 2) + pow (hBs - hUt, 2));
    }

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if hBS and hUT are within the validity range
  if (hUt < 1.5 || hUt >= 10.0)
    {
      NS_LOG_WARN ("The height of the UT should be between 1.5 and 22.5 m (see TR 38.901, Table 7.4.1-1). We further assume hUT < hBS, then hUT is upper bounded by hBS, which should be 10 m");
    }

  if (hBs != 10.0)
    {
      NS_LOG_WARN ("The height of the BS should be equal to 10 m (see TR 38.901, Table 7.4.1-1)");
    }

  // NOTE The model is intended to be used for BS-UT links, however we may need to
  // compute the pathloss between two BSs or UTs, e.g., to evaluate the
  // interference. In order to apply the model, we need to retrieve the values of
  // hBS and hUT, but in these cases one of the two falls outside the validity
  // range and the warning message is printed (hBS for the UT-UT case and hUT
  // for the BS-BS case).

  // check if the distace is outside the validity range
  if (distance2D < 10.0 || distance2D > 5.0e3)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 22.4 + 35.3 * log10 (distance3D) + 21.3 * log10 (m_frequency / 1e9) - 0.3 * (hUt - 1.5);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);
  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

std::pair<double, double>
ThreeGppUmiStreetCanyonPropagationLossModel::GetUtAndBsHeights (double za, double zb) const
{
  NS_LOG_FUNCTION (this);
  // TR 38.901 specifies hBS = 10 m and 1.5 <= hUT <= 22.5
  double hBs, hUt;
  if (za == 10.0)
    {
      // node A is the BS and node B is the UT
      hBs = za;
      hUt = zb;
    }
  else if (zb == 10.0)
    {
      // node B is the BS and node A is the UT
      hBs = zb;
      hUt = za;
    }
  else
    {
      // We cannot know who is the BS and who is the UT, we assume that the
      // tallest node is the BS and the smallest is the UT
      hBs = std::max (za, zb);
      hUt = std::min (za, za);
    }

  return std::pair<double, double> (hUt, hBs);
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 7.82;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double correlationDistance;

  // See 3GPP TR 38.901, Table 7.5-6
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 13;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  double PL_tw = 0;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return PL_tw;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  double stdP;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return stdP;
}

double
ThreeGppUmiStreetCanyonPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppIndoorOfficePropagationLossModel);

TypeId
ThreeGppIndoorOfficePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppIndoorOfficePropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppIndoorOfficePropagationLossModel> ()
  ;
  return tid;
}
ThreeGppIndoorOfficePropagationLossModel::ThreeGppIndoorOfficePropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppIndoorOpenOfficeChannelConditionModel> ();
}

ThreeGppIndoorOfficePropagationLossModel::~ThreeGppIndoorOfficePropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppIndoorOfficePropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);
  NS_UNUSED (distance3D);
  NS_UNUSED (hUt);
  NS_UNUSED (hBs);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 150.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 32.4 + 17.3 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 150.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 17.3 + 38.3 * log10 (distance3D) + 24.9 * log10 (m_frequency / 1e9);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 3.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 8.03;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  // See 3GPP TR 38.901, Table 7.5-6
  double correlationDistance;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 6;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  //En principio, indoorOffice no debería tener perdidas O2I según el estándar
  //Los desarrolladores del módulo NR si lo tenian en cuenta, lo dejo así como ellos.
  double PL_tw = 0;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          PL_tw = 5 - 10 * log10 (0.3 * pow (10,-1 * (2 + 0.2 * m_frequency * 1e-9) / 10) + 0.7 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return PL_tw;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);

  double stdP;

  if (a1->IsIndoor () && !b1->IsIndoor ())
    {
      if (a1->GetBuilding ()->GetBuildingType () == Building::Commercial || a1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }
      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else if (b1->IsIndoor () && !a1->IsIndoor ())
    {
      if (b1->GetBuilding ()->GetBuildingType () == Building::Commercial || b1->GetBuilding ()->GetBuildingType () == Building::Office)
        {
          NS_LOG_DEBUG ("Commercial and office building use high-loss model for UMa and UMi, use low-loss model for RMa");
          stdP = 6.5;
        }

      else
        {
          NS_LOG_DEBUG ("Residential building use low-loss model");
          stdP = 4.4;
        }

    }
  else
    {
      NS_FATAL_ERROR ("Programming Error");
    }
  return stdP;
}

double
ThreeGppIndoorOfficePropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppIndoorFactorySLPropagationLossModel);

TypeId
ThreeGppIndoorFactorySLPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppIndoorFactorySLPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppIndoorFactorySLPropagationLossModel> ()
  ;
  return tid;
}
ThreeGppIndoorFactorySLPropagationLossModel::ThreeGppIndoorFactorySLPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppIndoorFactorySLChannelConditionModel> ();
}

ThreeGppIndoorFactorySLPropagationLossModel::~ThreeGppIndoorFactorySLPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);
  NS_UNUSED (distance3D);
  NS_UNUSED (hUt);
  NS_UNUSED (hBs);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 31.84 + 21.5 * log10 (distance3D) + 19.0 * log10 (m_frequency / 1e9);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 33 + 25.5 * log10 (distance3D) + 20 * log10 (m_frequency / 1e9);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 5.7;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  // See 3GPP TR 38.901, Table 7.5-6 //Valor SF
  double correlationDistance;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 10;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  // Only the high-loss model is applicable to InF
  double PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
  return PL_tw;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  return 6.5;
}

double
ThreeGppIndoorFactorySLPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppIndoorFactoryDLPropagationLossModel);

TypeId
ThreeGppIndoorFactoryDLPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppIndoorFactoryDLPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppIndoorFactoryDLPropagationLossModel> ()
  ;
  return tid;
}
ThreeGppIndoorFactoryDLPropagationLossModel::ThreeGppIndoorFactoryDLPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppIndoorFactoryDLChannelConditionModel> ();
}

ThreeGppIndoorFactoryDLPropagationLossModel::~ThreeGppIndoorFactoryDLPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);
  NS_UNUSED (distance3D);
  NS_UNUSED (hUt);
  NS_UNUSED (hBs);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 31.84 + 21.5 * log10 (distance3D) + 19.0 * log10 (m_frequency / 1e9);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 18.6 + 35.7 * log10 (distance3D) + 20.0 * log10 (m_frequency / 1e9);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 7.2;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  // See 3GPP TR 38.901, Table 7.5-6 //Valor SF
  double correlationDistance;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 10;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  // Only the high-loss model is applicable to InF
  double PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
  return PL_tw;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  return 6.5;
}

double
ThreeGppIndoorFactoryDLPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppIndoorFactorySHPropagationLossModel);

TypeId
ThreeGppIndoorFactorySHPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppIndoorFactorySHPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppIndoorFactorySHPropagationLossModel> ()
  ;
  return tid;
}
ThreeGppIndoorFactorySHPropagationLossModel::ThreeGppIndoorFactorySHPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppIndoorFactorySHChannelConditionModel> ();
}

ThreeGppIndoorFactorySHPropagationLossModel::~ThreeGppIndoorFactorySHPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);
  NS_UNUSED (distance3D);
  NS_UNUSED (hUt);
  NS_UNUSED (hBs);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 31.84 + 21.5 * log10 (distance3D) + 19.0 * log10 (m_frequency / 1e9);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 32.4 + 23 * log10 (distance3D) + 20 * log10 (m_frequency / 1e9);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 5.9;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  // See 3GPP TR 38.901, Table 7.5-6 //Valor SF
  double correlationDistance;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 10;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  // Only the high-loss model is applicable to InF
  double PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
  return PL_tw;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  return 6.5;
}

double
ThreeGppIndoorFactorySHPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}
// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppIndoorFactoryDHPropagationLossModel);

TypeId
ThreeGppIndoorFactoryDHPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppIndoorFactoryDHPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppIndoorFactoryDHPropagationLossModel> ()
  ;
  return tid;
}
ThreeGppIndoorFactoryDHPropagationLossModel::ThreeGppIndoorFactoryDHPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // set a default channel condition model
  m_channelConditionModel = CreateObject<ThreeGppIndoorFactoryDHChannelConditionModel> ();
}

ThreeGppIndoorFactoryDHPropagationLossModel::~ThreeGppIndoorFactoryDHPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (distance2D);
  NS_UNUSED (distance3D);
  NS_UNUSED (hUt);
  NS_UNUSED (hBs);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss (see 3GPP TR 38.901, Table 7.4.1-1)
  double loss = 31.84 + 21.5 * log10 (distance3D) + 19.0 * log10 (m_frequency / 1e9);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  // check if the distace is outside the validity range
  if (distance3D < 1.0 || distance3D > 600.0)
    {
      NS_LOG_WARN ("The 2D distance is outside the validity range, the pathloss value may not be accurate");
    }

  // compute the pathloss
  double plNlos = 33.63 + 21.9 * log10 (distance3D) + 20 * log10 (m_frequency / 1e9);
  double loss = std::max (GetLossLos (distance2D, distance3D, hUt, hBs), plNlos);

  NS_LOG_DEBUG ("Loss " << loss);

  return loss;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a);
  NS_UNUSED (b);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = 4.0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 4.0;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  // See 3GPP TR 38.901, Table 7.5-6 //Valor SF
  double correlationDistance;

  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 10;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetPlTw (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  // Only the high-loss model is applicable to InF
  double PL_tw = 5 - 10 * log10 (0.7 * pow (10,-1 * (23 + 0.3 * m_frequency * 1e-9) / 10) + 0.3 * pow (10,-1 * (5 + 4 * m_frequency * 1e-9) / 10));
  return PL_tw;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetO2IStdDeviation (Ptr<MobilityBuildingInfo> a1, Ptr<MobilityBuildingInfo> b1) const
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (a1);
  NS_UNUSED (b1);

  return 6.5;
}

double
ThreeGppIndoorFactoryDHPropagationLossModel::GetDistance2dIn () const
{
  NS_LOG_FUNCTION (this);
  //compute PL_in
  Ptr<UniformRandomVariable> uniRv1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> uniRv2 = CreateObject<UniformRandomVariable> ();

  return (std::min (uniRv1->GetValue (0,25), uniRv2->GetValue (0,25)));
}

} // namespace ns3
