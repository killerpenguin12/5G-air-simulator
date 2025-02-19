/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of 5G-air-simulator
 *
 * 5G-air-simulator is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * 5G-air-simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 5G-air-simulator; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Telematics Lab <telematics-dev@poliba.it>
 */



#include "Application.h"
#include "../radio-bearer.h"
#include "../../device/NetworkNode.h"
#include "../../device/MulticastDestination.h"
#include "../QoS/QoSParameters.h"
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer-sink.h"
#include "application-sink.h"
#include "../../phy/phy.h"
#include "../../channel/RadioChannel.h"
#include "../../device/UserEquipment.h"
#include "../../device/GNodeB.h"
#include "../../load-parameters.h"
#include "../../protocolStack/application/application-entity.h"
#include "../../protocolStack/mac/random-access/ue-enhanced-random-access.h"

Application::Application()
{
  m_classifierParameters = nullptr;
  m_qosParameters = nullptr;
  m_source = nullptr;
  m_destination = nullptr;
  m_radioBearer = nullptr;
  m_bearerSink = nullptr;
  m_applicationSink = nullptr;
}

void
Application::Start ()
{
DEBUG_LOG_START_1(SIM_ENV_TEST_START_APPLICATION)
  cout << "Start Application: src: " << GetSource ()->GetIDNetworkNode ()
            << " dst: " << GetDestination ()->GetIDNetworkNode () << endl;
DEBUG_LOG_END

  // 1 - create radio bearer

  if (GetSource ()->GetNodeType() == NetworkNode::TYPE_UE)
    {
      //create an UL radio bearer between UE and targetGNB
      UserEquipment* ue = (UserEquipment*) GetSource ();

DEBUG_LOG_START_1(SIM_ENV_TEST_START_APPLICATION)
      cout << "Create UL radio bearer bewtween: " << GetSource ()->GetIDNetworkNode ()
                << " and " << ue->GetTargetNode ()->GetIDNetworkNode () << endl;
DEBUG_LOG_END

      m_radioBearer = new RadioBearer ();
      m_radioBearer->GetRlcEntity ()->SetRlcEntityIndex (GetApplicationID ());
      m_radioBearer->SetSource (ue);
      m_radioBearer->SetDestination (ue->GetTargetNode ());
      m_radioBearer->SetClassifierParameters (GetClassifierParameters ());
      m_radioBearer->SetApplication (this);
      m_radioBearer->SetQoSParameters (GetQoSParameters ());
      m_radioBearer->GetSource ()->GetProtocolStack ()->GetRrcEntity ()->AddRadioBearer (m_radioBearer);
    }
  else if (GetSource ()->GetNodeType() == NetworkNode::TYPE_GW
           ||
           GetSource ()->GetNodeType() == NetworkNode::TYPE_GNODEB
           ||
           GetSource ()->GetNodeType() == NetworkNode::TYPE_HOME_BASE_STATION)
    {
      if (GetDestination ()->GetNodeType() == NetworkNode::TYPE_UE)
        {
          //create an DL radio bearer between targetGNB and UE
          UserEquipment* ue = (UserEquipment*) GetDestination ();
          ue->MakeActive();

DEBUG_LOG_START_1(SIM_ENV_TEST_START_APPLICATION)
          cout << "Create DL radio bearer bewtween: " << ue->GetTargetNode ()->GetIDNetworkNode ()
                    << " and " << ue->GetIDNetworkNode ()  << endl;
DEBUG_LOG_END

          m_radioBearer = new RadioBearer ();
          m_radioBearer->GetRlcEntity ()->SetRlcEntityIndex (GetApplicationID ());
          m_radioBearer->SetSource (ue->GetTargetNode ());
          m_radioBearer->SetDestination (ue);
          m_radioBearer->SetClassifierParameters (GetClassifierParameters ());
          m_radioBearer->SetApplication (this);
          m_radioBearer->SetQoSParameters (GetQoSParameters ());
          m_radioBearer->GetSource ()->GetProtocolStack ()->GetRrcEntity ()->AddRadioBearer (m_radioBearer);
        }
      else if (GetDestination ()->GetNodeType() == NetworkNode::TYPE_MULTICAST_DESTINATION)
        {
          for (auto gnb:GetTransmittingNodes())
            {
              //create an DL radio bearer between targetGNB and UE
              UserEquipment* ue = (UserEquipment*) GetDestination ();
              ue->MakeActive();

DEBUG_LOG_START_1(SIM_ENV_TEST_START_APPLICATION)
              cout << "Create DL radio bearer bewtween: " << gnb->GetIDNetworkNode ()
                        << " and " << ue->GetIDNetworkNode ()  << endl;
DEBUG_LOG_END

              RadioBearer* bearer = new RadioBearer ();
              bearer->GetRlcEntity ()->SetRlcEntityIndex (GetApplicationID ());
              bearer->SetSource (gnb);
              bearer->SetDestination (ue);
              bearer->SetClassifierParameters (GetClassifierParameters ());
              bearer->SetApplication (this);
              bearer->SetQoSParameters (GetQoSParameters ());
              bearer->GetSource ()->GetProtocolStack ()->GetRrcEntity ()->AddRadioBearer (bearer);
              AddRadioBearer(bearer);
              if (ue->GetTargetNode() == gnb)
                {
                  m_radioBearer = bearer;
                }
            }
        }
    }


  // 2 - create application sink
  m_applicationSink = new ApplicationSink ();
  m_applicationSink->SetClassifierParameters (GetClassifierParameters ());
  m_applicationSink->SetSourceApplication (this);


  // 3 - create radio bearer sink and attach UE on the UL or DL channel
  m_bearerSink = new RadioBearerSink ();
  m_bearerSink->GetRlcEntity ()->SetRlcEntityIndex (GetApplicationID ());
  m_bearerSink->SetApplication (m_applicationSink);
  m_bearerSink->SetClassifierParameters (GetClassifierParameters ());
  m_bearerSink->SetQoSParameters (GetQoSParameters ());
  if (GetSource ()->GetNodeType() == NetworkNode::TYPE_UE)
    {
      UserEquipment* ue = (UserEquipment*) GetSource ();
      m_bearerSink->SetSource (ue);
      m_bearerSink->SetDestination (ue->GetTargetNode ());
    }
  else if (GetSource ()->GetNodeType() == NetworkNode::TYPE_GW || GetSource ()->GetNodeType() == NetworkNode::TYPE_GNODEB
           || GetSource ()->GetNodeType() == NetworkNode::TYPE_HOME_BASE_STATION)
    {
      UserEquipment* ue = (UserEquipment*) GetDestination ();
      m_bearerSink->SetSource (ue->GetTargetNode ());
      m_bearerSink->SetDestination (ue);

      RadioChannel *ch = ue->GetTargetNode ()->GetPhy ()->GetDlChannel ();
      RadioChannel *mcch = ue->GetTargetNode ()->GetPhy ()->GetDlMcChannel ();

      if (!ch->IsAttached (ue))
        {
          ch->AddDevice (ue);
        }
      else if (mcch != nullptr)
        {
          if (!mcch->IsAttached (ue) )
            {
              mcch->AddDevice (ue);
            }
        }
      ue->MakeActive();
    }


  // 4 - add in radio bearer a pointer to the radio bearer sink
  m_radioBearer->GetDestination() ->GetProtocolStack ()->GetRrcEntity ()->AddRadioBearerSink(m_bearerSink);
  m_applicationSink->SetRadioBearerSink (m_bearerSink);

  // create application and bearer sinks for users receiving multicast application
  if (GetDestination()->GetNodeType() == NetworkNode::TYPE_MULTICAST_DESTINATION)
    {
      MulticastDestination* virtualDest = (MulticastDestination*)GetDestination();
      for( auto ue : virtualDest->GetDestinations() )
        {
          ApplicationSink* appSink = new ApplicationSink ();
          appSink->SetClassifierParameters (GetClassifierParameters ());
          appSink->SetSourceApplication (this);

          RadioBearerSink* bearerSink = new RadioBearerSink ();
          bearerSink->GetRlcEntity ()->SetRlcEntityIndex (GetApplicationID ());
          bearerSink->SetApplication (appSink);
          bearerSink->SetClassifierParameters (GetClassifierParameters ());
          bearerSink->SetQoSParameters (GetQoSParameters ());

          bearerSink->SetSource (ue->GetTargetNode ());
          bearerSink->SetDestination (ue);

          ue->GetProtocolStack ()->GetRrcEntity ()->AddRadioBearerSink(bearerSink);
          appSink->SetRadioBearerSink (bearerSink);
        }
    }


DEBUG_LOG_START_1(SIM_ENV_TEST_START_APPLICATION)
  cout << "CREATED RADIO BEARER " << m_radioBearer->GetApplication ()->GetApplicationID ()
            << " BETWEEN "
            << m_radioBearer->GetSource ()->GetIDNetworkNode () << " and "
            << m_radioBearer->GetDestination () ->GetIDNetworkNode ()<< endl;
DEBUG_LOG_END

  DoStart ();
}

void
Application::Stop ()
{}


void
Application::enqueue (Packet* p)
{
  if (GetDestination ()->GetNodeType() == NetworkNode::TYPE_MULTICAST_DESTINATION)
    {
      for (auto bearer:m_radioBearers)
        {
          bearer->Enqueue (p->Copy());
        }
      delete p;
    }
  else
    {
      GetRadioBearer()->Enqueue (p);
    }
}

RadioBearer*
Application::GetRadioBearer (void)
{
  return m_radioBearer;
}

void
Application::SetApplicationID (int id)
{
  m_applicationID = id;
}

int
Application::GetApplicationID (void)
{
  return m_applicationID;
}


void
Application::SetApplicationType (ApplicationType applicationType)
{
  m_applicationType = applicationType;
}

Application::ApplicationType
Application::GetApplicationType (void) const
{
  return m_applicationType;
}

void
Application::SetClassifierParameters (ClassifierParameters* cp)
{
  m_classifierParameters = cp;
}

ClassifierParameters*
Application::GetClassifierParameters (void)
{
  return m_classifierParameters;
}

void
Application::SetQoSParameters (QoSParameters* qos)
{
  m_qosParameters = qos;
}

QoSParameters*
Application::GetQoSParameters (void)
{
  return m_qosParameters;
}

NetworkNode*
Application::GetSource (void)
{
  return m_source;
}

NetworkNode*
Application::GetDestination (void)
{
  return m_destination;
}

int
Application::GetSourcePort (void) const
{
  return m_sourcePort;
}

int
Application::GetDestinationPort (void) const
{
  return m_destinationPort;
}

void
Application::SetSource (NetworkNode *source)
{
  m_source = source;
  source->GetProtocolStack()->GetApplicationEntity()->InsertApplicationSource(this);
}

void
Application::SetDestination (NetworkNode *destination)
{
  m_destination = destination;
  destination->GetProtocolStack()->GetApplicationEntity()->InsertApplicationDestinations(this);
}

void
Application::SetSourcePort (int port)
{
  m_sourcePort = port;
}

void
Application::SetDestinationPort (int port)
{
  m_destinationPort = port;
}

TransportProtocol::TransportProtocolType
Application::GetTransportProtocol (void) const
{
  return m_transportProtocol;
}

void
Application::SetTransportProtocol (TransportProtocol::TransportProtocolType protocol)
{
  m_transportProtocol = protocol;
}

vector<NetworkNode*>
Application::GetTransmittingNodes(void)
{
  return m_transmittingNodes;
}

void
Application::SetTransmittingNodes(vector<NetworkNode*> nodes)
{
  m_transmittingNodes = nodes;
}

void
Application::AddTransmittingNode(NetworkNode* node)
{
  m_transmittingNodes.push_back(node);
}

vector<RadioBearer*>
Application::GetRadioBearers(void)
{
  return m_radioBearers;
}

void
Application::SetRadioBearers(vector<RadioBearer*> bearers)
{
  m_radioBearers = bearers;
}

void
Application::AddRadioBearer(RadioBearer* bearer)
{
  m_radioBearers.push_back(bearer);
}

void
Application::SetStartTime (double time)
{
  m_startTime = time;
  Simulator::Init()->Schedule(time,
                              &Application::Start,
                              this);
}

double
Application::GetStartTime (void) const
{
  return m_startTime;
}

void
Application::SetStopTime (double time)
{
  m_stopTime = time;
  Simulator::Init()->Schedule(time + 0.1,
                              &Application::Stop,
                              this);
}

double
Application::GetStopTime (void) const
{
  return m_stopTime;
}

void
Application::Trace (Packet* p)
{

  if (!_APP_TRACING_) return;

  /*
   * Trace format:
   *
   * TX   APPLICATION_TYPE   BEARER_ID  SIZE   SRC_ID   DST_ID   TIME
   */
  cout << "TX";
  switch (m_applicationType)
    {
    case Application::APPLICATION_TYPE_VOIP:
      {
        cout << " VOIP";
        break;
      }
    case Application::APPLICATION_TYPE_TRACE_BASED:
      {
        cout << " VIDEO";
        break;
      }
    case Application::APPLICATION_TYPE_CBR:
      {
        cout << " CBR";
        break;
      }
    case Application::APPLICATION_TYPE_INFINITE_BUFFER:
      {
        cout << " INF_BUF";
        break;
      }
    case Application::APPLICATION_TYPE_FTP2:
      {
        cout << " FTP";
        break;
      }
    case Application::APPLICATION_TYPE_EXTERNAL_SOURCE:
      {
        cout << " EXTERNAL_SOURCE";
        break;
      }
    default:
      {
        cout << " UNDEFINED";
        break;
      }
    }

  if (GetDestination ()->GetNodeType() == NetworkNode::TYPE_UE)
    {
      UserEquipment* ue = (UserEquipment*) GetDestination ();
      cout << " ID here" << p->GetID ()
                << " B " << GetApplicationID ()
                << " SIZE " << p->GetSize ()
                << " SRC " << GetSource ()->GetIDNetworkNode ()
                << " DST " << GetDestination ()->GetIDNetworkNode ()
                << " T " << Simulator::Init()->Now()
                << " " << ue->IsIndoor () << endl;
      UeMacEntity* mac = ue->GetMacEntity();
      Simulator::Init()->Schedule(0, &UeRandomAccess::StartRaProcedure, mac->GetRandomAccessManager());

    }
  else
    {
      cout << " ID here two" << p->GetID ()
                << " B " << GetApplicationID ()
                << " SIZE " << p->GetSize ()
                << " SRC " << GetSource ()->GetIDNetworkNode ()
                << " DST " << GetDestination ()->GetIDNetworkNode ()
                << " T " << Simulator::Init()->Now() << endl;
      if (GetDestination ()->GetNodeType()!= NetworkNode::TYPE_MULTICAST_DESTINATION)
      {
    	  UserEquipment* ue = (UserEquipment*) GetSource ();
    	  UeMacEntity* mac = ue->GetMacEntity();
    	  Simulator::Init()->Schedule(0, &UeRandomAccess::StartRaProcedure, mac->GetRandomAccessManager());
      }
    }
}

void
Application::Print (void)
{
  cout << " Application object: "
       "\n\t m_applicationType = " << m_applicationType <<
       endl;
}

