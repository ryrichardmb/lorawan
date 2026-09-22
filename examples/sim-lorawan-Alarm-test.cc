/*
 * =====================================================================================
 *
 *       Filename:  lorawan-network-wAlm-sim.cc
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  12/07/2026 13:42:04
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Francisco Helder (FHC), helderhdw@gmail.com
 *   Organization:  Federal University of Ceara
 *
 * =====================================================================================
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/random-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("LorawanNetworkAlmSimulator");

#define MAXRTX 4


// Network settings
uint16_t nDevices = 200;
uint16_t nRegulars = 150;
uint16_t nAlarms = 100;
uint8_t nGateways = 1;
uint16_t radius = 6400; //Note that due to model updates, 7500 m is no longer the maximum distance 
double gatewayRadius = 0;
uint16_t simulationTime = 3600;

// Channel model
bool realisticChannelModel = false;

uint16_t appPeriodSeconds = 60;

// Output control
bool printBuildings = false;
bool printED = false;

enum SF { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  printEndDevices
 *  Description:  
 * =====================================================================================
 */
void PrintEndDevices (NodeContainer endDevices, NodeContainer gateways, std::string filename1, std::string filename2, std::string filename3){
  	const char * c = filename1.c_str ();
  	std::ofstream spreadingFactorFile;
  	spreadingFactorFile.open (c);
	// print for regular event
  	for (int j = 0; j < nRegulars; ++j){
    	Ptr<Node> object = endDevices.Get(j);
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	NS_ASSERT (position);
      	Ptr<NetDevice> netDevice = object->GetDevice (0);
      	Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      	NS_ASSERT (loraNetDevice);
      	Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      	int sf = int(mac->GetSfFromDataRate(mac->GetDataRate ()));
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " " << sf << endl;
  	}
	spreadingFactorFile.close ();
  	
  	c = filename2.c_str ();
  	spreadingFactorFile.open (c);
	// print for alarm event
  	for (int j = nRegulars; j < nDevices; ++j){
    	Ptr<Node> object = endDevices.Get(j);
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	NS_ASSERT (position);
      	Ptr<NetDevice> netDevice = object->GetDevice (0);
      	Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      	NS_ASSERT (loraNetDevice);
      	Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      	int sf = int(mac->GetSfFromDataRate(mac->GetDataRate ()));
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " " << sf << endl;
  	}
  	spreadingFactorFile.close ();
  	
	c = filename3.c_str ();
  	spreadingFactorFile.open (c);	
  	// Also print the gateways
  	for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j){
    	Ptr<Node> object = *j;
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " GW" << endl;
  	}
  	spreadingFactorFile.close ();
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  buildingHandler
 *  Description:  
 * =====================================================================================
 */
void buildingHandler ( NodeContainer endDevices, NodeContainer gateways ){

	double xLength = 230;
  	double deltaX = 80;
  	double yLength = 164;
  	double deltaY = 57;
 	int gridWidth = 2 * radius / (xLength + deltaX);
  	int gridHeight = 2 * radius / (yLength + deltaY);

  	if (realisticChannelModel == false){
    	gridWidth = 0;
    	gridHeight = 0;
    }
  
	Ptr<GridBuildingAllocator> gridBuildingAllocator;
  	gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  	gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  	gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  	gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  	gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  	gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  	gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  	gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  	gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  	gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  	gridBuildingAllocator->SetAttribute (
      "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  	gridBuildingAllocator->SetAttribute (
      "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  	BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  	BuildingsHelper::Install (endDevices);
  	BuildingsHelper::Install (gateways);
    //BuildingsHelper::MakeMobilityModelConsistent ();

  	// Print the buildings
  	if (printBuildings){
    	std::ofstream myfile;
    	myfile.open ("buildings.txt");
      	std::vector<Ptr<Building>>::const_iterator it;
      	int j = 1;
      	for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j){
			Box boundaries = (*it)->GetBoundaries ();
        	myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin
                 << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
      	}
      	myfile.close ();
    }

}/* -----  end of function buildingHandler  ----- */

vector<uint16_t> CountSfInRange (NodeContainer endDevices, uint16_t first, uint16_t last){
	vector<uint16_t> count (6, 0);
	for (uint16_t j = first; j < last; ++j){
		Ptr<Node> node = endDevices.Get (j);
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
		int sf = int (mac->GetSfFromDataRate (mac->GetDataRate ()));
		count.at (sf - SF7)++;
	}
	return count;
}

int main (int argc, char *argv[]){
	string fileMetric="./TestResult/test";
 	string fileData="./TestResult/test";
	string fileRegMetric="./TestResult/test";
	string fileAlmMetric="./TestResult/test";
	string endDevRegFile="./TestResult/test";
	string endDevAlmFile="./TestResult/test";
	string gwFile="./TestResult/test";
	bool flagRtxReg=false,flagRtxAlm=false;
  	uint32_t nSeed=1;
	uint8_t trial=1, numClass=0;
	vector<uint16_t> sfQuantAlm(6,0), sfQuantReg(6,0);
	vector<uint16_t> sfQuant(6,0);
	double packLoss=0, sent=0, received=0, avgDelay=0;
	double angle=0, sAngle=M_PI;
	double throughput=0, probSucc=0, probLoss=0; 

 
  	CommandLine cmd;
  	cmd.AddValue ("nSeed", "Number of seed to position", nSeed);
  	cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
	cmd.AddValue ("nAlarm", "Number of end devices to include in the simulation", nAlarms);
  	cmd.AddValue ("nGateways", "Number of gateway rings to include", nGateways);
  	cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  	cmd.AddValue ("gatewayRadius", "The distance between gateways", gatewayRadius);
  	cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  	cmd.AddValue ("appPeriod", "The period in seconds to be used by periodically transmitting applications", appPeriodSeconds);
	cmd.AddValue ("flagRtxAlm", "Enable (1) or disable (0) retransmission alarm", flagRtxAlm);
	cmd.AddValue ("flagRtxReg", "Enable (1) or disable (0) retransmission regular", flagRtxReg);
  	cmd.AddValue ("print", "Whether or not to print various informations", printED);
  	cmd.AddValue ("trial", "set trial parameter", trial);
  	cmd.Parse (argc, argv);

	bool flagRtx = (flagRtxAlm || flagRtxReg);

	/*******************************
 	 * Regulars and Alarms 	Define *
 	 *******************************/

	//nAlarms = nDevices/(10);
	nRegulars = nDevices - nAlarms;

	NS_LOG_DEBUG("number regular event: " << nRegulars << " number alarm event: " << nAlarms );
 
	//endDevFile += to_string(trial) + "/endDevices" + to_string(nDevices) + ".dat";
	endDevRegFile += to_string(trial) + "/endDevicesReg" + to_string(nRegulars) + ".dat";
	endDevAlmFile += to_string(trial) + "/endDevicesAlm" + to_string(nAlarms) + ".dat";
	gwFile += to_string(trial) + "/GWs" + to_string(nGateways) + ".dat";
	
	fileMetric += to_string(trial) + "/traffic-" + to_string(appPeriodSeconds) + "/result-STAs";
 	fileData   += to_string(trial) + "/traffic-" + to_string(appPeriodSeconds) + "/mac-STAs-GW-" + to_string(nGateways);
 	fileRegMetric += to_string(trial) + "/traffic-" + to_string(appPeriodSeconds) + "/result-Reg-STAs";
 	fileAlmMetric += to_string(trial) + "/traffic-" + to_string(appPeriodSeconds) + "/result-Alm-STAs";

	string trafficDir = "./TestResult/test" + to_string(trial) + "/traffic-" + to_string(appPeriodSeconds) + "/";
	string dirAllSF = trafficDir + "metricAllSF/";
	string dirRegSF = trafficDir + "metricRegSF/";
	string dirAlmSF = trafficDir + "metricAlmSF/";

	//TODO:Retirar após validação
	
	std::error_code ec;
	for (const string &d : {dirAllSF, dirRegSF, dirAlmSF})
	{
		std::filesystem::create_directories (d, ec);
		if (ec) NS_LOG_ERROR ("Falha ao criar " << d << ": " << ec.message ());
	}
	

 	// Set up logging
  	 LogComponentEnable ("LorawanNetworkAlmSimulator", LOG_LEVEL_ALL);
  	// LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
  	// LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  	// LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  	// LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
   	// LogComponentEnable("SimpleEndDeviceLoraPhy", LOG_LEVEL_DEBUG);
  	// LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
   	// LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_DEBUG);
  	// LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
  	// LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  	// LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  	// LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  	// LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  	// LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
   	// LogComponentEnable("RandomSenderHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable("RandomSender", LOG_LEVEL_ALL);
  	// LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
  	// LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  	// LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
  	// LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
  	// LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
  	// LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

  	/***********
   	*  Setup  *
   	***********/
	ofstream myfile;
  	RngSeedManager::SetSeed(1);
  	RngSeedManager::SetRun(nSeed);

  	// Create the time value from the period
  	Time appPeriod = Seconds (appPeriodSeconds);

 	// Mobility
  	MobilityHelper mobility;
  	mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
     	                            "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  	/************************
   	*  Create the channel  *
   	************************/

  	// Create the lora channel object
  	Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  	loss->SetPathLossExponent (3.76);
  	loss->SetReference (1, 7.7);

  	if (realisticChannelModel){
  		// Create the correlated shadowing component
      	Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
        	  CreateObject<CorrelatedShadowingPropagationLossModel> ();

      	// Aggregate shadowing to the logdistance loss
      	loss->SetNext (shadowing);

      	// Add the effect to the channel propagation loss
      	Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      	shadowing->SetNext (buildingLoss);
    }

  	Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  	Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  	/************************
   	*  Create the helpers  *
   	************************/

  	// Create the LoraPhyHelper
  	LoraPhyHelper phyHelper = LoraPhyHelper ();
  	phyHelper.SetChannel (channel);

  	// Create the LorawanMacHelper
  	LorawanMacHelper macHelper = LorawanMacHelper ();

  	// Create the LoraHelper
  	LoraHelper helper = LoraHelper ();
  	helper.EnablePacketTracking (); // Output filename
  	// helper.EnableSimulationTimePrinting ();

  	//Create the NetworkServerHelper
  	NetworkServerHelper nsHelper = NetworkServerHelper ();

  	//Create the ForwarderHelper
  	ForwarderHelper forHelper = ForwarderHelper ();

  	/************************
   	*  Create End Devices  *
   	************************/

  	// Create a set of nodes
  	NodeContainer endDevices;
  	endDevices.Create (nDevices);

  	// Assign a mobility model to each node
  	mobility.Install (endDevices);

  	// Make it so that nodes are at a certain height > 0
  	for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j){
      	Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      	Vector position = mobility->GetPosition ();
      	position.z = 1.2;
      	mobility->SetPosition (position);
	}

  	// Create the LoraNetDevices of the end devices
  	uint8_t nwkId = 54;
  	uint32_t nwkAddr = 1864;
  	Ptr<LoraDeviceAddressGenerator> addrGen =
     	 CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  	// Create the LoraNetDevices of the end devices
  	macHelper.SetAddressGenerator (addrGen);
  	phyHelper.SetDeviceType (LoraPhyHelper::ED);
  	macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  	helper.Install (phyHelper, macHelper, endDevices);

  	// Now end devices are connected to the channel
	// Connect trace sources
  	for (uint16_t j=0; j<nRegulars; ++j){
    	Ptr<Node> node = endDevices.Get(j);
      	Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      	Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
	
		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac>();
      	if (flagRtxReg){
	  		mac->SetMaxNumberOfTransmissions (MAXRTX);
	  		mac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
	  	}
   	}

  	for (uint16_t j=nRegulars; j<nDevices; ++j){
    	Ptr<Node> node = endDevices.Get(j);
      	Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      	Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
	
		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac>();
      	if (flagRtxAlm){
	  		mac->SetMaxNumberOfTransmissions (MAXRTX);
	  		mac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
	  	}
   	}

 
  	/*********************
   	*  Create Gateways  *
   	*********************/

  	// Create the gateway nodes (allocate them uniformely on the disc)
  	NodeContainer gateways;
  	gateways.Create (nGateways);

  	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  	// Make it so that nodes are at a certain height > 0
  	allocator->Add (Vector (0.0, 0.0, 15.0));
  	mobility.SetPositionAllocator (allocator);
  	mobility.Install (gateways);

  	// Make it so that nodes are at a certain height > 0
  	for (NodeContainer::Iterator j = gateways.Begin ();
    	j != gateways.End (); ++j){
      	Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      	Vector position = mobility->GetPosition ();
		position.x = gatewayRadius * cos(angle); 
  		position.y = gatewayRadius * sin(angle); 
      	position.z = 15;
      	mobility->SetPosition (position);
		angle += sAngle;
	}


  	// Create a netdevice for each gateway
  	phyHelper.SetDeviceType (LoraPhyHelper::GW);
  	macHelper.SetDeviceType (LorawanMacHelper::GW);
  	helper.Install (phyHelper, macHelper, gateways);

  	/**********************************************
  	*  Set up the end device's spreading factor  *
   	**********************************************/
	/**********************
   	*  Handle buildings  *
   	**********************/
	buildingHandler(endDevices, gateways);	
 
  	/**********************************************
   	*  Set up the end device's spreading factor  *
   	**********************************************/
  	sfQuant = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

	//Contagem de SF de cada dispositivos (ALARM/REGULAR)
	sfQuantReg = CountSfInRange(endDevices, 0, nRegulars);
	sfQuantAlm = CountSfInRange(endDevices, nRegulars, nDevices);

	for(uint8_t i=0; i<sfQuant.size(); i++)
		sfQuant.at(i)?numClass++:numClass;

	NS_LOG_DEBUG ("Completed configuration");

  	/*********************************************
   	*  Install applications on the end devices  *
   	*********************************************/

  	Time appStopTime = Seconds (simulationTime);

	PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  	appHelper.SetPeriod (Seconds (appPeriodSeconds));
  	appHelper.SetPacketSize (19);
  	Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
    	  "Min", DoubleValue (0), "Max", DoubleValue (10));
  	ApplicationContainer appContainer = appHelper.Install (endDevices);


/*    RandomSenderHelper appHelper = RandomSenderHelper ();
  	appHelper.SetMean (appPeriodSeconds);
   	appHelper.SetPacketSize (19);
  	ApplicationContainer appContainer = appHelper.Install (endDevices);

  	appContainer.Start (Seconds (0));
  	appContainer.Stop (appStopTime);
*/
  	/**************************
   	*  Create Network Server  *
   	***************************/
    // Create the network server node
    Ptr<Node> networkServer = CreateObject<Node>();

    // PointToPoint links between gateways and server
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    // Store network server app registration details for later
    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(networkServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    // Create a network server for the network
    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.Install(networkServer);

  	//Create a forwarder for each gateway
  	forHelper.Install (gateways);

 	/**********************
   	* Print output files *
   	*********************/
  	if (printED){
    	PrintEndDevices (endDevices, gateways, endDevRegFile, endDevAlmFile, gwFile);
 	}

  	////////////////
  	// Simulation //
  	////////////////

  	Simulator::Stop (appStopTime + Hours (1));

  	NS_LOG_INFO ("Running simulation...");
  	Simulator::Run ();

  	Simulator::Destroy ();

 	NS_LOG_INFO("SF Allocation: "<< "SF7=" << (unsigned)sfQuant.at(0) << " SF8=" << (unsigned)sfQuant.at(1) << " SF9=" << (unsigned)sfQuant.at(2)
				<< " SF10=" << (unsigned)sfQuant.at(3) << " SF11=" << (unsigned)sfQuant.at(4) << " SF12=" << (unsigned)sfQuant.at(5));

	NS_LOG_INFO("SF Allocation (REGULAR): "<< "SF7=" << sfQuantReg.at(0) << " SF8=" << sfQuantReg.at(1) << " SF9=" << sfQuantReg.at(2)
				<< " SF10=" << sfQuantReg.at(3) << " SF11=" << sfQuantReg.at(4) << " SF12=" << sfQuantReg.at(5));

	NS_LOG_INFO("SF Allocation (ALARM)  : "<< "SF7=" << sfQuantAlm.at(0) << " SF8=" << sfQuantAlm.at(1) << " SF9=" << sfQuantAlm.at(2)
				<< " SF10=" << sfQuantAlm.at(3) << " SF11=" << sfQuantAlm.at(4) << " SF12=" << sfQuantAlm.at(5));


  	LoraPacketTracker &tracker = helper.GetPacketTracker ();
  
  	//////////////////////////////////
  	// Print global results to file //
  	//////////////////////////////////
 
  	stringstream(tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1))) >> sent >> received;
	
	avgDelay = 0;
	if(flagRtx)
		stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0), appStopTime + Hours(1), (unsigned)nDevices, (unsigned)nGateways)) >> avgDelay;

	packLoss = sent - received;
  	throughput = received/simulationTime;

  	probSucc = received/sent;
  	probLoss = packLoss/sent;

	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
   	NS_LOG_INFO("nDevices: " << nDevices); 
	NS_LOG_INFO("thrghput: " << throughput); 
	NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
	NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
	NS_LOG_INFO("avgDelay: " << avgDelay); 
	NS_LOG_INFO("----------------------------------"<< endl);

  	myfile.open (fileMetric+".dat", ios::out | ios::app);
  	myfile << nDevices << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  	myfile.close();  

   	NS_LOG_INFO("numDev:" << nDevices << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  	NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

  	myfile.open (fileData, ios::out | ios::app);
  	myfile << "sent: " << sent << " succ: " << received << " drop: "<< packLoss << "\n";
  	myfile << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << "\n";
  	myfile << "numDev: " << nDevices << " numGat: " << nGateways << " simTime: " << simulationTime << " throughput: " << throughput<< "\n";
  	myfile << "##################################################" << "\n\n";
  	myfile.close();

	//SF ALL
	for(uint8_t i=SF7;i<SF7+numClass;i++)
	{
		stringstream(tracker.CountMacPacketsGlobally(Seconds (0), appStopTime + Hours (1), i)) >> sent >> received;

		avgDelay = 0;
		if(flagRtx)
			stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0), appStopTime + Hours(1), (unsigned)nDevices, (unsigned)nGateways, i)) >> avgDelay;

		packLoss = sent - received;
  		throughput = received/simulationTime;

  		probSucc = received/sent;
  		probLoss = packLoss/sent;

		NS_LOG_INFO(">> Computing SF-"<<(unsigned)i<<" All performance metrics >>");
   		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
   		NS_LOG_INFO("nDevices: " << nDevices); 
		NS_LOG_INFO("thrghput: " << throughput); 
		NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
		NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
		NS_LOG_INFO("avgDelay: " << avgDelay); 
		NS_LOG_INFO("----------------------------------"<< endl);

		myfile.open (dirAllSF+"result-STAs-all-SF"+to_string(i)+".dat", ios::out | ios::app);
  		myfile << nDevices << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  		myfile.close();

		NS_LOG_INFO("numDev:" << nDevices << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  		NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

	}

  	/////////////////////////////////
  	// Print alarm results to file //
  	/////////////////////////////////
 
  	stringstream(tracker.CountMacPacketsForType (Seconds (0), appStopTime + Hours (1), ALARM, (unsigned)nRegulars, (unsigned)nDevices)) >> sent >> received;
	
	avgDelay = 0;
	if(flagRtxAlm)
    stringstream(tracker.CountMacPacketsForTypeDelay(Seconds(0), appStopTime + Hours(1), ALARM,
                 (unsigned)nRegulars, (unsigned)nDevices,
                 (unsigned)nDevices, (unsigned)nGateways)) >> avgDelay;

	packLoss = sent - received;
  	throughput = received/simulationTime;

  	probSucc = received/sent;
  	probLoss = packLoss/sent;

	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
   	NS_LOG_INFO("Alarms  : " << nAlarms); 
	NS_LOG_INFO("thrghput: " << throughput); 
	NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
	NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
	NS_LOG_INFO("avgDelay: " << avgDelay); 
	NS_LOG_INFO("----------------------------------"<< endl);

  	myfile.open (fileAlmMetric+".dat", ios::out | ios::app);
  	myfile << nAlarms << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  	myfile.close();  

   	NS_LOG_INFO("numAlm:" << nAlarms << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  	NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

	//SF ALM
	for(uint8_t i=SF7;i<SF7+numClass;i++)
	{
		stringstream(tracker.CountMacPacketsForType (Seconds (0), appStopTime + Hours (1), ALARM, (unsigned)nRegulars, (unsigned)nDevices, i)) >> sent >> received;
		
		avgDelay = 0;
		if(flagRtxAlm)
    	stringstream(tracker.CountMacPacketsForTypeDelay(Seconds(0), appStopTime + Hours(1), ALARM,
                 (unsigned)nRegulars, (unsigned)nDevices,
                 (unsigned)nDevices, (unsigned)nGateways, i)) >> avgDelay;

		packLoss = sent - received;
  		throughput = received/simulationTime;

  		probSucc = received/sent;
  		probLoss = packLoss/sent;

		NS_LOG_INFO(">> Computing SF-"<<(unsigned)i<<" Alarm performance metrics >>");
   		NS_LOG_INFO("Alarms  : " << nAlarms); 
		NS_LOG_INFO("thrghput: " << throughput); 
		NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
		NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
		NS_LOG_INFO("avgDelay: " << avgDelay); 
		NS_LOG_INFO("----------------------------------"<< endl);

		myfile.open (dirAlmSF+"result-STAs-alarm-SF"+to_string(i)+".dat", ios::out | ios::app);
  		myfile << nAlarms << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  		myfile.close();

		NS_LOG_INFO("numAlm:" << nAlarms << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  		NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

	}

  	///////////////////////////////////
  	// Print regular results to file //
  	///////////////////////////////////
 
  	stringstream(tracker.CountMacPacketsForType (Seconds (0), appStopTime + Hours (1), REGULAR, (unsigned)nRegulars, (unsigned)nDevices)) >> sent >> received;
	
	avgDelay = 0;
	if(flagRtxReg)
    stringstream(tracker.CountMacPacketsForTypeDelay(Seconds(0), appStopTime + Hours(1), REGULAR,
                 (unsigned)nRegulars, (unsigned)nDevices,
                 (unsigned)nDevices, (unsigned)nGateways)) >> avgDelay;

	packLoss = sent - received;
  	throughput = received/simulationTime;

  	probSucc = received/sent;
  	probLoss = packLoss/sent;

	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
   	NS_LOG_INFO("Regulars: " << nRegulars); 
	NS_LOG_INFO("thrghput: " << throughput); 
	NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
	NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
	NS_LOG_INFO("avgDelay: " << avgDelay); 
	NS_LOG_INFO("----------------------------------"<< endl);

  	myfile.open (fileRegMetric+".dat", ios::out | ios::app);
  	myfile << nRegulars << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  	myfile.close();  

   	NS_LOG_INFO("numReg:" << nRegulars << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  	NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

	//SF REG
	for(uint8_t i=SF7;i<SF7+numClass;i++)
	{
		stringstream(tracker.CountMacPacketsForType (Seconds (0), appStopTime + Hours (1), REGULAR, (unsigned)nRegulars, (unsigned)nDevices, i)) >> sent >> received;

		avgDelay = 0;
		if(flagRtxReg)
    	stringstream(tracker.CountMacPacketsForTypeDelay(Seconds(0), appStopTime + Hours(1), REGULAR,
                 (unsigned)nRegulars, (unsigned)nDevices,
                 (unsigned)nDevices, (unsigned)nGateways, i)) >> avgDelay;

		packLoss = sent - received;
  		throughput = received/simulationTime;

  		probSucc = received/sent;
  		probLoss = packLoss/sent;

		NS_LOG_INFO(">> Computing SF-"<<(unsigned)i<<" Regular performance metrics >>");
   		NS_LOG_INFO("Regulars: " << nRegulars); 
		NS_LOG_INFO("thrghput: " << throughput); 
		NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
		NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss*100 << "%)"); 
		NS_LOG_INFO("avgDelay: " << avgDelay); 
		NS_LOG_INFO("----------------------------------"<< endl);

		myfile.open (dirRegSF+"result-STAs-reg-SF"+to_string(i)+".dat", ios::out | ios::app);
  		myfile << nRegulars << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
  		myfile.close();

		NS_LOG_INFO("numReg:" << nRegulars << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  		NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  		NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);

	}

  	return(0);
}
