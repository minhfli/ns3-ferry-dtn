/*
 * DTN Base Framework for NS-3
 * Focus: Store-Carry-Forward Logic
 * Mode: 802.11b (adhoc) - No Association, Immediate Communication
 * Ferries move in a fixed TSP route through all ground nodes
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

#include "ferry_app/base-dtn-app.h"
#include "ferry_app/simple-dtn-app.h"
#include "ferry_app/pigeon-dtn-app.h"
#include "ferry_app/tabaf-dtn-app.h"
#include "ferry_app/sr-pigeon-dtn-app.h"
#include "ferry_app/sr-pigeon-v2-dtn-app.h"
#include "ferry_app/tabadla-dtn-app.h"
#include "ferry_app/tabara-dtn-app.h"
#include "ferry_app/taba2s-dtn-app.h"
#include "ferry_app/route-prune-delay-aware-dtn-app.h"
#include "ferry_app/multi-route-pigeon-dtn-app.h"
#include "ferry_app/sr-tabaf-dtn-app.h"
#include "ferry_app/divided-coupling-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <set>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FerryDtnSimulation");

Ptr<BaseDtnApp> createApp() {
    if (config.ALGORITHM_NAME == "SIRA") {
        return CreateObject<SingleRouteDtnApp>();
    }
    if (config.ALGORITHM_NAME == "PIGEON") {
        return CreateObject<PigeonDtnApp>();
    }
    if (config.ALGORITHM_NAME == "TABAF") {
        return CreateObject<TabafDtnApp>();
    }
    if (config.ALGORITHM_NAME == "SR_PIGEON") {
        return CreateObject<SingleRoutePigeonDtnApp>();
    }
    if (config.ALGORITHM_NAME == "SR_PIGEON_V2") {
        return CreateObject<SingleRoutePigeonV2DtnApp>();
    }
    if (config.ALGORITHM_NAME == "TABADLA") {
        return CreateObject<TabaDlaDtnApp>();
    }
    if (config.ALGORITHM_NAME == "TABARA") {
        return CreateObject<TabaraDtnApp>();
    }
    if (config.ALGORITHM_NAME == "TABA2S") {
        return CreateObject<Taba2sDtnApp>();
    }
    if (config.ALGORITHM_NAME == "RPDLAS") {
        return CreateObject<RoutePrunningDeadlineAwareShortcutDtnApp>();
    }
    if (config.ALGORITHM_NAME == "MRDLAS") {
        return CreateObject<MultiRouteDeadlineAwareShortcutDtnApp>();
    }
    if (config.ALGORITHM_NAME == "SR_TABAF") {
        return CreateObject<SingleRouteTabafDtnApp>();
    }
    if (config.ALGORITHM_NAME == "DRC") {
        return CreateObject<DividedRouteCouplingDtnApp>();
    }

    return nullptr;
}

uint32_t GetAlgoGroupCount() {
    if (config.ALGORITHM_NAME == "SIRA") return 1;
    if (config.ALGORITHM_NAME == "PIGEON") return config.nFerrys;
    if (config.ALGORITHM_NAME == "TABAF") return 1;
    if (config.ALGORITHM_NAME == "SR_PIGEON") return 1;
    if (config.ALGORITHM_NAME == "SR_PIGEON_V2") return 1;
    if (config.ALGORITHM_NAME == "TABADLA") return 1;
    if (config.ALGORITHM_NAME == "TABARA")  return 1;
    if (config.ALGORITHM_NAME == "TABA2S")  return 1;
    if (config.ALGORITHM_NAME == "RPDLAS")  return 1;
    if (config.ALGORITHM_NAME == "SR_TABAF")  return 1;
    if (config.ALGORITHM_NAME == "MRDLAS")  return 1;
    if (config.ALGORITHM_NAME == "DRC")  return config.nFerrys;
    return 1;
}

std::vector<std::vector<uint32_t>> GetAlgoClusters() {
    if (config.ALGORITHM_NAME == "PIGEON") return Clustering::TSPAidedBisect(groundNodePos, config.nFerrys);
    if (config.ALGORITHM_NAME == "DRC") return Clustering::TSPAidedBisect(groundNodePos, config.nFerrys);

    std::vector<std::vector<uint32_t>> clusters;
    clusters.push_back(DataStructureHelper::GetIndexVector(config.nGrounds));
    return clusters;
}

void CallGlobalFerryAppSetup(const std::vector<std::vector<uint32_t>>& clusters) {
    NS_LOG_UNCOND("Calling global ferry setup for: " << config.ALGORITHM_NAME);
    if (config.ALGORITHM_NAME == "RPDLAS") {
        RPDLAS::FerrySetup();
        return;
    }
    if (config.ALGORITHM_NAME == "MRDLAS") {
        MRDLAS::FerrySetup();
        return;
    }
    if (config.ALGORITHM_NAME == "DRC") {
        DRC::FerrySetup(clusters);
        return;
    }
    return;
}

void CallLogAdditionalInfo(std::string filename) {
    if (config.ALGORITHM_NAME == "DRC") {
        DRC::LogAdditionalInfo(filename);
        return;
    }
}

// ===========================================================================
// MAIN SCRIPT
// ===========================================================================
int main(int argc, char* argv[]) {
    // ==========================================================
    // Loggings //! IMPORTANT 
    // ==========================================================
    // parse cmd line args
    ParseConfig(argc, argv);

    FerryVisualizer::vizFileName = "/mnt/d/coding/python/dtn-visualizer/trace/" + config.REPORT_BATCH + "/" + config.ALGORITHM_NAME + "_" + config.SIMULATION_RUN + ".log";
    FerryVisualizer::vizAdditionalFileName = "/mnt/d/coding/python/dtn-visualizer/" + config.ALGORITHM_NAME + "_" + config.SIMULATION_RUN + ".txt";
    Report::reportFileName = "/mnt/d/coding/python/dtn-visualizer/report/" + config.REPORT_BATCH + "/" + config.ALGORITHM_NAME + "_" + config.SIMULATION_RUN + ".log";

    if (config.skipIfExist) {
        std::ifstream reportFile(Report::reportFileName);
        if (reportFile.good()) {
            std::cout << "Report file already exists, skipping simulation." << std::endl;
            return 0;
        }
    }
    LogComponentEnable("FerryDtnSimulation", LOG_LEVEL_INFO);

    srand(config.randSeed);
    SeedManager::SetSeed(config.randSeed);
    // initialize global random generator
    m_rand = CreateObject<UniformRandomVariable>();
    bundleGenRand = CreateObject<UniformRandomVariable>();

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

    // Generate ground node positions
    double areaPadding = config.areaPadding; // padding to avoid node on edge of the map
    double rangePadding = config.commRange * 2.0 + 20; // padding tp avoid node near communication range
    groundNodePos =
        PoissonDisk_RandomSample(config.nGrounds, rangePadding, config.areaWidth - areaPadding * 2);

    // Cấu hình mobility cho ground node
    MobilityHelper groundMobility;
    Ptr<ListPositionAllocator> ground_lpa = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < config.nGrounds; i++) {
        groundNodePos[i].x += areaPadding;
        groundNodePos[i].y += areaPadding;
        ground_lpa->Add(Vector(groundNodePos[i].x, groundNodePos[i].y, 0));
    }
    groundMobility.SetPositionAllocator(ground_lpa);
    groundMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    groundMobility.Install(GroundNodes);

    // Generate ferry starting position
    auto ferryNodePos = PoissonDisk_RandomSample(config.nFerrys, 0, config.areaWidth - areaPadding * 2);

    // Cấu hình Ferry bay
    MobilityHelper ferryMobility;
    ferryMobility.SetPositionAllocator("ns3::ListPositionAllocator");
    Ptr<ListPositionAllocator> ferry_lpa = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < config.nFerrys; i++) {
        ferryNodePos[i].x += areaPadding;
        ferryNodePos[i].y += areaPadding;
        ferry_lpa->Add(Vector(ferryNodePos[i].x, ferryNodePos[i].y, config.ferryHeight));
    }
    ferryMobility.SetPositionAllocator(ferry_lpa);
    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // ==========================================================
    // INTERNET STACK 
    // ==========================================================

    InternetStackHelper internet;
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    // ==========================================================
    // Application
    // ==========================================================
    BundleGenerationHelper::Init();
    uint32_t groupCount = GetAlgoGroupCount();
    auto clusters = GetAlgoClusters();

    for (uint32_t n = 0; n < config.nGrounds; n++) {
        groundNodeIps.push_back(i.GetAddress(n));
    }
    for (uint32_t group = 0; group < groupCount; group++) {
        auto& cluster = clusters[group];
        // Cài App cho Ground Nodes
        for (auto n : cluster) {
            Ptr<Node> gNode = GroundNodes.Get(n);
            Ptr<Socket> socket = Socket::CreateSocket(gNode, tid);
            Ptr<BaseDtnApp> app = createApp();
            Ipv4Address address = i.GetAddress(gNode->GetId());

            nodeType[address.Get()] = NODE_TYPE_GROUND;
            nodeId[address.Get()] = "g" + std::to_string(gNode->GetId());
            nodeGroup[address.Get()] = group;

            gNode->AddApplication(app);
            app->Setup(gNode, socket, address, config.groundBufferSize, NODE_TYPE_GROUND);
            app->SetStartTime(Seconds(1.0));
            app->SetStopTime(Seconds(config.simTime + config.warmupTime));
            app->SetGroup(group);
            app->EnableBundleGeneration(groundGenRate[n], true);
        }
    }

    // Cài App cho Ferry (Index trong container i là config.nGrounds)
    CallGlobalFerryAppSetup(clusters);

    for (uint32_t n = 0; n < config.nFerrys; n++) {
        Ptr<Node> fNode = ferryNode.Get(n);

        Ptr<Socket> socket = Socket::CreateSocket(fNode, tid);
        Ptr<BaseDtnApp> app = createApp();
        Ipv4Address address = i.GetAddress(fNode->GetId());

        nodeType[address.Get()] = NODE_TYPE_FERRY;
        nodeId[address.Get()] = "f" + std::to_string(fNode->GetId());
        ferryIps.push_back(address);

        if (groupCount == 1) {
            nodeGroup[address.Get()] = 0;
        }
        else {
            nodeGroup[address.Get()] = n % groupCount;
        }

        fNode->AddApplication(app);
        app->Setup(fNode, socket, address, config.ferryBufferSize, NODE_TYPE_FERRY);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime + config.warmupTime));
        app->SetGroup(nodeGroup[address.Get()]);
        app->InitializeMobility(clusters[nodeGroup[address.Get()]]);
    }
    // ==========================================================
    // Simulation
    // ==========================================================
    Report::Init();
    FerryVisualizer::SetUp();
    Simulator::Stop(Seconds(config.simTime + config.warmupTime)); // Đã set trong app
    Simulator::Run();
    Simulator::Destroy();
    FerryVisualizer::CleanUp();
    Report::Export();
    CallLogAdditionalInfo(FerryVisualizer::vizAdditionalFileName);

    return 0;
}