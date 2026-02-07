/*
 * Implementation of https://ieeexplore.ieee.org/document/8566956 in ns3
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/wave-module.h" 
#include "ns3/point-to-point-module.h"

#include "ferry_helper/global.h"
#include "ferry_helper/config.h"
#include "ferry_helper/report.h"

#include "ferry_helper/ferry-helper.h"
#include "ferry_helper/tsp-helper.h"
#include "ferry_helper/cluster-helper.h"
#include "ferry_helper/packet-helper.h"
#include "ferry_helper/viz-helper.h"

#include "ferry_app/pigeon-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleDtnApp");

// ===========================================================================
// MAIN SCRIPT
// ===========================================================================
int main(int argc, char* argv[]) {
    // set random seed so every run is the same
    srand(1337);
    SeedManager::SetSeed(1337);
    // initialize global random generator
    m_rand = CreateObject<UniformRandomVariable>();

    // ==========================================================
    // Loggings //! IMPORTANT 
    // ==========================================================


    // parse cmd line args
    ParseConfig(argc, argv);

    FerryVisualizer::vizFileName = "/mnt/d/coding/python/dtn-visualizer/log/dtn-" + config.SIMULATION_NAME + "_" + config.SIMULATION_RUN + ".log";
    Report::reportFileName = "/mnt/d/coding/python/dtn-visualizer/log/report-" + config.SIMULATION_NAME + "_" + config.SIMULATION_RUN + ".log";

    LogComponentEnable("SimpleDtnApp", LOG_LEVEL_INFO);

    // ==========================================================
    // Tạo Node
    // ==========================================================

    NodeContainer allNodes;
    NodeContainer GroundNodes;
    GroundNodes.Create(config.nGrounds);
    NodeContainer ferryNode;
    ferryNode.Create(config.nFerrys);
    allNodes.Add(GroundNodes);
    allNodes.Add(ferryNode);

    simVar.nGrounds = config.nGrounds;
    simVar.nFerrys = config.nFerrys;
    simVar.groundNodes = &GroundNodes;
    simVar.ferryNodes = &ferryNode;

    // ==========================================================
    // CẤU HÌNH WIFI (802.11b)
    // ==========================================================

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel;

    // Dùng mô hình đơn giản, quan trọng là routing logic
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(config.commRange));
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);

    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue("DsssRate11Mbps"),
                                "ControlMode", StringValue("DsssRate11Mbps"),
                                "NonUnicastMode", StringValue("DsssRate11Mbps"),
                                "RtsCtsThreshold", StringValue("0"));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, allNodes);

    // ==========================================================
    // MOBILITY
    // ==========================================================

    double areaPadding = 100; // padding to avoid node on edge of the map
    double rangePadding = config.commRange + 30; // padding tp avoid node near communication range
    groundNodePos = PoissonDisk_RandomSample(
        config.nGrounds, config.commRange + rangePadding, config.areaWidth - areaPadding);

    std::vector<std::vector<uint32_t>> groundNodeClusters = KMeans(groundNodePos, config.nFerrys);

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> ground_lpa = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < config.nGrounds; i++) {
        groundNodePos[i].x += areaPadding / 2;
        groundNodePos[i].y += areaPadding / 2;
        ground_lpa->Add(Vector(groundNodePos[i].x, groundNodePos[i].y, 0));
    }
    mobility.SetPositionAllocator(ground_lpa);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(GroundNodes);

    // Cấu hình Ferry bay
    MobilityHelper ferryMobility;
    ferryMobility.SetPositionAllocator("ns3::ListPositionAllocator");
    Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
    lpa->Add(Vector(config.areaWidth / 2, config.areaWidth / 2, config.ferryHeight)); // Bắt đầu tại chính giữa, cao 50m
    ferryMobility.SetPositionAllocator(lpa);

    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // ==========================================================
    // INTERNET STACK & APP
    // ==========================================================

    InternetStackHelper internet;
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    // Cài App & group cho Ground Nodes
    int group = 0;
    for (auto cluster : groundNodeClusters) {
        for (auto n : cluster) {

            Ptr<Node> gNode = GroundNodes.Get(n);
            Ptr<Socket> socket = Socket::CreateSocket(gNode, tid);
            Ptr<PigeonDtnApp> app = CreateObject<PigeonDtnApp>();
            Ipv4Address address = i.GetAddress(gNode->GetId());

            groundNodeIps.push_back(address);
            nodeGroup[address.Get()] = group;

            gNode->AddApplication(app);
            app->Setup(gNode, socket, address, config.groundBufferSize, NODE_TYPE_GROUND);
            app->SetStartTime(Seconds(1.0));
            app->SetStopTime(Seconds(config.simTime));
            app->EnableBundleGeneration(config.bundleGenRate);
            app->SetGroup(group);

            nodeType[address.Get()] = NODE_TYPE_GROUND;
            nodeId[address.Get()] = "g" + std::to_string(gNode->GetId());
        }
        group++;
    }



    // Cài App cho Ferry (Index trong container i là config.nGrounds)
    for (uint32_t n = 0; n < config.nFerrys; n++) {
        Ptr<Node> fNode = ferryNode.Get(n);

        Ptr<Socket> socket = Socket::CreateSocket(fNode, tid);
        Ptr<PigeonDtnApp> app = CreateObject<PigeonDtnApp>();
        Ipv4Address address = i.GetAddress(fNode->GetId());

        fNode->AddApplication(app);
        app->Setup(fNode, socket, address, config.ferryBufferSize, NODE_TYPE_FERRY);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime));

        app->SetGroup(n); // set group id for ferry

        nodeType[address.Get()] = NODE_TYPE_FERRY;
        nodeId[address.Get()] = "f" + std::to_string(fNode->GetId());
        nodeGroup[address.Get()] = n;

        // ===== Set up base TSP Mobility ===== 
        std::vector<point2D> points;
        std::vector<uint32_t> cluster = groundNodeClusters[n];
        for (auto index : cluster)
            points.push_back(groundNodePos[index]);
        std::vector<uint32_t> order = TSPClassicGA(points);
        order = TSPTwoOptOptimize(points, order);
        app->InitializeTSPMobility(points, order);
    }

    // ==========================================================
    // Simulation
    // ==========================================================
    FerryVisualizer::SetUp();
    Simulator::Stop(Seconds(config.simTime)); // Đã set trong app
    Simulator::Run();
    Simulator::Destroy();
    FerryVisualizer::CleanUp();
    Report::Export();

    return 0;
}