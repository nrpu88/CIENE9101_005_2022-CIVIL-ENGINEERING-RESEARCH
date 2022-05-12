#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/traci-applications-module.h"
#include "ns3/network-module.h"
#include "ns3/traci-module.h"
#include "ns3/wave-module.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/netanim-module.h"
// #include "../../../millicar/helper/mmwave-vehicular-net-device.h"

// /home/ubuntu/Downloads/XihangYu_NS3_SUMO_Coupling/ns-allinone-3.28/ns3-mmwave/src/traci-applications/examples/ns3-sumo-coupling-simple.cc

// #include "./ns-allinone-3.28/ns3-mmwave/src/millicar/helper/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/config.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/applications-module.h"
#include <functional>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;
ofstream myfile;


using namespace ns3;
using namespace millicar;
NS_LOG_COMPONENT_DEFINE("ns3-sumo-coupling-simple");

uint32_t g_rxPackets; // total number of received packets
uint32_t g_txPackets; // total number of transmitted packets

Time g_firstReceived; // timestamp of the first time a packet is received
Time g_lastReceived; // timestamp of the last received packet
Ptr<TraciClient> sumoClient = CreateObject<TraciClient> (); // create sumo client
NodeContainer nodePool; // create the node pool
uint32_t port = 4000;
//UdpClientHelper client (nodePool.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);

/*

All the statements/changes for the addition of the 3rd vehicle are commented with double quotes(""). All these changes were made by Nripesh Kumar (MS CS 2022).
You can reach out to me at: nk2913@columbia.edu

*/

static void Rx (Ptr<OutputStreamWrapper> stream, UdpClientHelper *client, Ptr<const Packet> p)
{
 g_rxPackets++;
 SeqTsHeader header;
 MobilityHeader VelocityHeader;

 //peek headers
 p->PeekHeader(header);
 p->PeekHeader(VelocityHeader);

 *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds() << std::endl;
 std::cout << "Time: " << Simulator::Now ().GetSeconds () << " Size: " << p->GetSize() << " Seq: " << header.GetSeq() << " Time: " << header.GetTs().GetSeconds() << std::endl;
 std::cout << "Velocity is coming: " << VelocityHeader.GetVelocity() << '\n';
 // front car (node)
 std::cout << "Velocity of the front car:" << sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(nodePool.Get (0))) << "m/s" << '\n';
 std::cout << "Position of the front node:" << nodePool.Get (0)->GetObject<MobilityModel> ()->GetPosition () <<'\n';
 // get vehicle position from sumo
 libsumo::TraCIPosition pos0(sumoClient->TraCIAPI::vehicle.getPosition(sumoClient->GetVehicleId(nodePool.Get (0))));
 std::cout << "Sumo position of the front car: " << Vector(pos0.x, pos0.y, 1.5) << std::endl;
 
 // set velocity in the header of packets
 uint32_t vel=sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(nodePool.Get (0)));
 std::cout << "Vel in Rx: " << vel << std::endl;
 client->SetVelocity(vel);
 std::cout << "1. You are in Rx: " << vel << '\n';


 //get lane information
 /*
 Steps for sending out warning message because of bad lane(possible implementation):
 1. All information about current state of the lane can be obtained by using TraCI API calls: https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
 2. We need to forcefully make a lane/edge unusable. This needs to be explicitly set in the SUMO route file (.sumo.cfg file)
 3. The warning message can then be encoded into the UDP packet header. This can be done by setting up a flag inside the UDP packet. 
    If flag = 1 then the lane is bad.
    If flag = 0 then the lane is good.
 */
 string currLaneID = sumoClient->TraCIAPI::vehicle.getLaneID(sumoClient->GetVehicleId(nodePool.Get (0)));
 cout<<"The edgeID of the front vehicle is: "<<currLaneID;
 
 // get velocity from header
 double velocity = (double) VelocityHeader.GetVelocity();
 std::cout << "2. You are in Rx: " << velocity << '\n';
 std::cout << sumoClient->GetVehicleId (nodePool.Get (0)) << '\n';
 sumoClient->TraCIAPI::vehicle.setSpeed (sumoClient->GetVehicleId (nodePool.Get (1)), velocity);
 std::cout << "3. You are in Rx: " << vel << '\n';

 // following car
 std::cout << "Velocity of the following car:" << sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(nodePool.Get (1))) << "m/s" << '\n';
 std::cout << "Position of the following node:" << nodePool.Get (1)->GetObject<MobilityModel> ()->GetPosition () <<'\n';
 std::cout << "4. You are in Rx: " << vel << '\n';
 libsumo::TraCIPosition pos1(sumoClient->TraCIAPI::vehicle.getPosition(sumoClient->GetVehicleId(nodePool.Get (1))));
 std::cout << "Sumo position of the following car: " << Vector(pos1.x, pos1.y, 1.5) << std::endl;
 std::cout << "5. You are in Rx: " << vel << '\n';

 // input velocity information into text scripts
 myfile.open("/home/ubuntu/Downloads/XihangYu_NS3_SUMO_Coupling/ns-allinone-3.28/ns3-mmwave/src/traci-applications/examples/velocity_follow.txt", std::ios_base::app);
 float velfollowing = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(nodePool.Get (1)));
 float round_velfollowing = roundf(velfollowing * 100) / 100;
 myfile << round_velfollowing << ' ';
 myfile.close();

 myfile.open("/home/ubuntu/Downloads/XihangYu_NS3_SUMO_Coupling/ns-allinone-3.28/ns3-mmwave/src/traci-applications/examples/velocity_front.txt", std::ios_base::app);
 float velfront = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(nodePool.Get (0)));
 float round_velfront = roundf(velfront * 100) / 100;
 myfile << round_velfront << ' ';
 myfile.close();

 myfile.open("/home/ubuntu/Downloads/XihangYu_NS3_SUMO_Coupling/ns-allinone-3.28/ns3-mmwave/src/traci-applications/examples/velocity_time.txt", std::ios_base::app);
 float Velocitytime = Simulator::Now ().GetSeconds ();
 float round_Velocitytime = roundf(Velocitytime * 100000000) / 100000000;
 myfile << round_Velocitytime << ' ';
 myfile.close();

 if (g_rxPackets > 1)
 {

   g_lastReceived = Simulator::Now();
 }
 else
 {
   g_firstReceived = Simulator::Now();
 }
}



int
main (int argc, char *argv[])
{
  /*** 0. Logging Options ***/
  bool verbose = true;

  if (verbose)
    {
      LogComponentEnable ("TraciClient", LOG_LEVEL_INFO);
      LogComponentEnable ("TrafficControlApplication", LOG_LEVEL_INFO);
    }
  
  // coding here
  
  
  // system parameters
  double bandwidth = 1e8; // bandwidth in Hz
  double frequency = 28e9; // the carrier frequency
  uint32_t numerology = 3; // the numerology

  // applications
  uint32_t packetSize = 1024; // UDP packet size in bytes
  //uint32_t startTime = 5; // application start time in seconds
  ns3::Time simulationTime (ns3::Seconds(3)); // application end time in seconds
  uint32_t interPacketInterval = 30; // interpacket interval in microseconds

  // mobility
  //double speed = 50; // speed of the vehicles m/s
  double intraGroupDistance = 10; // distance between two vehicles belonging to the same group

  CommandLine cmd;
  
  cmd.AddValue ("bandwidth", "used bandwidth", bandwidth);
  cmd.AddValue ("iip", "inter packet interval, in microseconds", interPacketInterval);
  cmd.AddValue ("intraGroupDistance", "distance between two vehicles belonging to the same group, y-coord", intraGroupDistance);
  cmd.AddValue ("numerology", "set the numerology to use at the physical layer", numerology);
  cmd.AddValue ("frequency", "set the carrier frequency", frequency);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::Numerology", UintegerValue (numerology));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue ("a"));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::Shadowing", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveVehicularSpectrumPropagationLossModel::UpdatePeriod", TimeValue (MilliSeconds (1)));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElements", UintegerValue (16));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElementPattern", StringValue ("3GPP-V2V"));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::IsotropicAntennaElements", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::NumSectors", UintegerValue (2));

  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (500*1024));



  /*** 1. Create node pool and counter; large enough to cover all sumo vehicles ***/
  // ns3::Time simulationTime (ns3::Seconds(20));
  //NodeContainer nodePool;

  nodePool.Create (3);   // "Changed the number of nodes from 2->3"
  uint32_t nodeCounter (0);

  /*** 2. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  
  mobility.Install (nodePool);
  
  nodePool.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  nodePool.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0, intraGroupDistance,  0));
  nodePool.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 2*intraGroupDistance,  0)); // "Set the spawn point of the 3rd vehicle"
  
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper> ();
  helper->SetNumerology (3);
  helper->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  NetDeviceContainer netDevices = helper->InstallMmWaveVehicularNetDevices (nodePool);



  std::cout << "1 Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  
  /*** 3. Add Internet layers stack and routing ***/
  InternetStackHelper stack;
  stack.Install (nodePool);

  /*** 4. Assign IP address to each device ***/
  Ipv4AddressHelper address;
  NS_LOG_INFO("Assign IP Adresses.");
  address.SetBase ("10.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer ipv4Interfaces;
  ipv4Interfaces = address.Assign (netDevices);


  // Don't change this part  
  std::cout << "2 Packets size:\t\t" << packetSize << " Bytes" << std::endl;

  /*** 5. Setup Traci and start SUMO ***/
  //Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue ("src/traci/examples/circle-simple/manhattan_traffic_0.sumo.cfg"));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.001)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));  // portion of vehicles equipped with wifi
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  
  
  // Need to pair the devices in order to create a correspondence between transmitter and receiver
  // and to populate the < IP addr, RNTI > map.

  helper->PairDevices(netDevices);

  std::cout << "3 Packets size:\t\t" << packetSize << " Bytes" << std::endl;


  // 6. Set the routing table
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (nodePool.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (nodePool.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );
  staticRouting->SetDefaultRoute (nodePool.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 ); // "Added the default route for the 3rd vehicle"

  NS_LOG_DEBUG("IPv4 Address node 0: " << nodePool.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1: " << nodePool.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1: " << nodePool.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ()); // "Added for the 3rd vehicle"

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject <mmwave::MmWaveAmc> (helper->GetConfigurationParameters());

  // 7. setup the applications

  
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (0xFFFFFFFF));
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MicroSeconds (interPacketInterval)));
  Config::SetDefault ("ns3::UdpClient::PacketSize", UintegerValue (packetSize));
  //config::SetDefault ("ns3::UdpClient::Velocity", UintegerValue (30));  
 

  // 8. create the applications
  //uint32_t port = 4000;

  UdpEchoServerHelper server (port);
  ApplicationContainer echoApps = server.Install (nodePool.Get (1));
  ApplicationContainer echoApps1 = server.Install (nodePool.Get (2)); // "Defined new server for the 3rd vehicle"


  echoApps.Start (Seconds(0.45));
  echoApps1.Start (Seconds(0.75));
  // "Defined start time for the server in the 3rd vehicle. Note that this is DIFFERENT from the entry time of the 3rd vehicle in the simulation"
  // " The start time for the server is always set to be after the entry time of the vehicle"
  // " The entry time of the vehicle can be found in the 'manhattan_traffic.rou.xml' file"

  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("simple-one-stats.txt");

  UdpClientHelper client (nodePool.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
  UdpClientHelper client1 (nodePool.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port); // "Created new client for the 3rd vehicle"

  echoApps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream, &client)); // Rx is called after packets are received
  echoApps1.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream, &client1)); //"Added for the 3rd vehicle"
  //client.SetAttribute("NodePool", nodePool); 
  //client.SetAttribute("SumoClient", sumoClient); 
  //client.SetAttribute("Velocity", UintegerValue (50)); 
  //client.SetAttribute("Velocity", UintegerValue (30));  

  
  std::cout << "God" << '\n';
  ///
  //UdpTraceClient clientHelper;
  //clientHelper.SetAttribute("Velocity", UintegerValue (30));
  //clientHelper.Install(nodePool.Get (1))

  ApplicationContainer apps = client.Install (nodePool.Get (0));
  


  std::cout << "4 Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  
  /*** 9. Setup interface and application for dynamic nodes ***/
  /***
  VehicleSpeedControlHelper vehicleSpeedControlHelper (9); // why here is (9)? just the port name
  vehicleSpeedControlHelper.SetAttribute ("Client", (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  ***/


  /* MULTI-GROUP SCENARIO Possible implementation: https://github.com/signetlabdei/millicar
  In order to set up the scenario where multiple groups of vehicles are to be introduced, the following function needs to be modified as follows:-
  1. The current function only takes one Node as input. This needs to be modified so that the function takes one nodePool of data type 'NodeContainer'
     as input.
  2. After taking nodePool as input, each vehicle corresponding to a node inside the nodePool can be created.
  3. The exact same change needs to be made to the node shutdown function.
  */

  // callback function for node creation
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node>
    {
      std::cout << "9 Packets size:\t\t" << " Bytes" << std::endl;

      std::cout << "This is the number of node: " << nodeCounter << "\n";
      if (nodeCounter >= nodePool.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");
      

      // don't create and install the protocol stack of the node at simulation time -> take from "node pool"
      Ptr<Node> includedNode = nodePool.Get(nodeCounter);
      ++nodeCounter;// increment counter for next node

      // Install Application
      /***
      ApplicationContainer vehicleSpeedControlApps = vehicleSpeedControlHelper.Install (includedNode);
      vehicleSpeedControlApps.Start (Seconds(0.0));
      vehicleSpeedControlApps.Stop (simulationTime);
      ***/

      return includedNode;
    };

  // callback function for node shutdown
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode)
    {
      std::cout << "10 Packets size:\t\t" << " Bytes" << std::endl;
      // stop all applications
      Ptr<VehicleSpeedControl> vehicleSpeedControl = exNode->GetApplication(0)->GetObject<VehicleSpeedControl>();
      if(vehicleSpeedControl)
        vehicleSpeedControl->StopApplicationNow();

      // set position outside communication range
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-100.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      // NOTE: further actions could be required for a save shut down!
    };


  std::cout << "5 Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  // start traci client with given function pointers
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 10. Setup and Start Simulation + Animation ***/

  std::cout << "6 Packets size:\t\t" << packetSize << " Bytes" << std::endl;

  AnimationInterface anim ("src/traci-applications/examples/ns3-sumo-coupling.xml"); // Mandatory
  Simulator::Stop (simulationTime);
  std::cout << "7 Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  


  Simulator::Run ();
  
  Ptr<MobilityModel> mobilityNode0 = nodePool.Get (0)->GetObject<MobilityModel> ();
  ns3::Vector posCurr = mobilityNode0->GetPosition();
  std::cout << "Position: " << posCurr << std::endl;

  std::cout << "8 Packets size:\t\t" << packetSize << " Bytes" << std::endl;

  Simulator::Destroy ();

  std::cout << "----------- Statistics -----------" << std::endl;
  std::cout << "Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  std::cout << "Packets received:\t" << g_rxPackets << std::endl;
  std::cout << "Average Throughput:\t" << (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6 << " Mbps" << std::endl;

  return 0;
}
