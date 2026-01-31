/*
 * DTN Base Framework for NS-3 (OCB Version)
 * Focus: Store-Carry-Forward Logic (The ONE style)
 * Mode: 802.11p (WAVE/OCB) - No Association, Immediate Communication
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

#include "ferry_helper/config.h"
#include "ferry_helper/ferry-helper.h"
#include "ferry_helper/tsp-helper.h"
#include "ferry_helper/cluster-helper.h"
#include "ferry_helper/packet-helper.h"
#include "ferry_helper/viz-helper.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleDtnApp");

constexpr uint8_t NODE_TYPE_GROUND = 0;
constexpr uint8_t NODE_TYPE_FERRY = 1;

// ===========================================================================
// Global variables
// ===========================================================================

Ptr<UniformRandomVariable> m_rand;
std::vector<Ipv4Address> groundNodeIps;
// mapping from IP to type, all node have this infomation
std::unordered_map<uint32_t, uint8_t> nodeType;
// mapping from IP to group, all node have this infomation
std::unordered_map<uint32_t, uint8_t> nodeGroup;

// ===========================================================================
// 2. APP & BUNDLE STORAGE (Có cập nhật Jitter)
// ===========================================================================

class SimpleDtnApp : public Application
{
    public:
    SimpleDtnApp();
    virtual ~SimpleDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType);
    void EnableBundleGeneration(double rate, bool inversed = true);

    private:
    static Time GetJitter();

    virtual void StartApplication(void);
    virtual void StopApplication(void);

    // --- CORE LOGIC: Nơi bạn sẽ cài thuật toán Routing --)

    void ReceivePacket(Ptr<Socket> socket);
    // Sending function
    void Beacon();
    void SendGroundHello(Ipv4Address ferryIp);
    void SendFerryHello(Ipv4Address ferryIp);
    void SendFerryAcceptTransfer(Ipv4Address groundIp);
    void SendBundle(Bundle& bundle, Ipv4Address neighborIp);
    void SendBundleAck(Bundle bundle, Ipv4Address neighborIp);

    // This function is called by the ferry node
    // it will send all bundle to ground node then send an accept transfer message
    // allowing ground node to send message to it
    void ScheduleAcceptTransfer(Ipv4Address groundIp);
    // This function is called by the ground node when receive an ACK or AcceptTransfer from the ferry node
    // it will send bundle to ferry node 
    void ScheduleTransfer(Ipv4Address ferryIp);

    // Bundle Logic
    void GenerateBundle();
    void RemoveExpiredBundles(); // remove old bundle that over TTL
    void RemoveOldBundle(); // remove oldest bundles to keep buffer size < max buffer size
    void RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp); // remove transmitted bundle

    void BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp);


    void OnNeighborDiscovery(Ipv4Address neighborIp);
    bool ShouldForward(const Bundle& bundle, Ipv4Address neighborIp);

    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    // EventId m_helloEvent;

    double m_bundleGenRate;

    std::vector<Bundle> m_buffer;
    uint32_t m_maxBufferSize;
    uint32_t m_bundleIdCounter;
};

NS_OBJECT_ENSURE_REGISTERED(SimpleDtnApp);

TypeId SimpleDtnApp::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::SimpleDtnApp")
        .SetParent<Application>()
        .AddConstructor<SimpleDtnApp>();
    return tid;
}

SimpleDtnApp::SimpleDtnApp()
    : m_port(80), m_maxBufferSize(100), m_bundleIdCounter(0) {
}

SimpleDtnApp::~SimpleDtnApp() {}

void SimpleDtnApp::Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType)
{
    m_socket = socket;
    m_myIp = myIp;
    m_maxBufferSize = bufferSize;
    m_nodeType = nodeType;
    m_bundleGenRate = 0.0;

    m_buffer.clear();
    m_buffer.reserve(m_maxBufferSize + 1);
}

/**
 * inversed:
 *  true -> rate: average seconds between 2 bundle : seconds / packet
 *  false -> packet geration rate : packets/second
 */
void SimpleDtnApp::EnableBundleGeneration(double rate, bool inversed) {
    if (inversed) {
        m_bundleGenRate = 1.0 / rate;
    }
    else {
        m_bundleGenRate = rate;
    }
    double averageBundleTime = 1.0 / m_bundleGenRate;
    double startTime = 0.1 + m_rand->GetValue(0.0, averageBundleTime);
    Simulator::Schedule(Seconds(startTime), &SimpleDtnApp::GenerateBundle, this);
}

void SimpleDtnApp::StartApplication(void)
{
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port)) == -1)
    {
        NS_LOG_UNCOND("Bind failed for node " << m_myIp);
        return;
    }
    m_socket->SetRecvCallback(MakeCallback(&SimpleDtnApp::ReceivePacket, this));

    // OCB hỗ trợ broadcast rất tốt qua IP broadcast
    m_socket->SetAllowBroadcast(true);

    NS_LOG_UNCOND("Node " << m_myIp << " started listening on port " << m_port);

    if (m_nodeType == 0)
    {
        NS_LOG_UNCOND("Node " << m_myIp << " is a GROUND node.");
    }
    else
    {
        NS_LOG_UNCOND("Node " << m_myIp << " is a FERRY node.");


        Simulator::Schedule(GetJitter(), &SimpleDtnApp::Beacon, this);
    }
}

void SimpleDtnApp::StopApplication(void) {
    if (m_socket) { m_socket->Close(); }
}

Time SimpleDtnApp::GetJitter() {
    return MicroSeconds(m_rand->GetInteger(config.jitterAmount, config.jitterAmount * 3));
}

void SimpleDtnApp::Beacon() {
    // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Sending HELLO (OCB)");
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_BEACON);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, broadcast);

    Time nextBeacon = Seconds(m_rand->GetValue(0.0, config.beaconRandomness)) + Seconds(config.beaconInterval);
    // Schedule next Hello
    Simulator::Schedule(nextBeacon, &SimpleDtnApp::Beacon, this);
}

void SimpleDtnApp::SendGroundHello(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::GROUND_HELLO);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void SimpleDtnApp::SendFerryHello(Ipv4Address ferryIp) {
    // TODO
}

void SimpleDtnApp::SendFerryAcceptTransfer(Ipv4Address groundIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_ACCEPT_TRANSFER);
    header.SetNodeIP(m_myIp);

    FerryRouteHeader fRouteHeader; // TODO handling waypoints
    fRouteHeader.SetGroup(0);
    fRouteHeader.SetCount(0);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(groundIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void SimpleDtnApp::SendBundle(Bundle& bundle, Ipv4Address neighborIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::BUNDLE);
    header.SetNodeIP(m_myIp);

    BundleHeader bundleHeader;
    bundleHeader.SetHop(bundle.hop);
    bundleHeader.SetBundleId(bundle.id);
    bundleHeader.SetSourceIp(bundle.source);
    bundleHeader.SetDestIp(bundle.destination);
    bundleHeader.SetCreationTime(bundle.creationTime);

    bundle.flag_waitingAck = true;
    Simulator::Schedule(MicroSeconds(config.bundleAckTimeout),
        &SimpleDtnApp::BundleAckTimeout, this, bundle.id, bundle.source.Get());

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void SimpleDtnApp::SendBundleAck(Bundle bundle, Ipv4Address neighborIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::BUNDLE_ACK);
    header.SetNodeIP(m_myIp);

    BundleAckHeader bundleAckHeader;
    bundleAckHeader.SetBundleId(bundle.id);
    bundleAckHeader.SetSourceIp(bundle.source);
    bundleAckHeader.SetDestIp(neighborIp);
    bundleAckHeader.SetAllowSendNext(true);
    RemoveExpiredBundles();
    if (m_buffer.size() >= m_maxBufferSize) {
        bundleAckHeader.SetAllowSendNext(false);
    }

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleAckHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void SimpleDtnApp::ScheduleAcceptTransfer(Ipv4Address groundIp) {
    Time jitter = GetJitter();

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false && bundle.destination == groundIp) {
            Simulator::Schedule(jitter, &SimpleDtnApp::SendBundle, this, bundle, groundIp);
            return;
        }
    }
    RemoveExpiredBundles();
    if (m_buffer.size() >= m_maxBufferSize) {
        return; // there are no buffer left to receive additional bundle
    }
    Simulator::Schedule(jitter, &SimpleDtnApp::SendFerryAcceptTransfer, this, groundIp);
}

void SimpleDtnApp::ScheduleTransfer(Ipv4Address ferryIp) {
    Time jitter = GetJitter();

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) { // TODO implement multiple ferry with multiple ground group
            Simulator::Schedule(jitter, &SimpleDtnApp::SendBundle, this, bundle, ferryIp);
            return;
        }
    }
}


void SimpleDtnApp::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        MessageTypeHeader topHeader;
        packet->RemoveHeader(topHeader);

        // Tự mình gửi thì bỏ qua (Loopback)
        if (topHeader.GetNodeIP() == m_myIp) continue;

        auto packetType = topHeader.GetType();
        if (packetType == MessageTypeHeader::FERRY_BEACON) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << m_myIp
                << ": BEACON from Ferry " << topHeader.GetNodeIP());
            Time jitter = GetJitter();
            if (m_nodeType == NODE_TYPE_GROUND) {
                Simulator::Schedule(jitter, &SimpleDtnApp::SendGroundHello, this, topHeader.GetNodeIP());
            }
            continue;
        }
        if (packetType == MessageTypeHeader::GROUND_HELLO) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << m_myIp
                << ": HELLO from Ground node " << topHeader.GetNodeIP()
            );

            if (m_nodeType == NODE_TYPE_FERRY) {
                ScheduleAcceptTransfer(topHeader.GetNodeIP());
            }
            continue;
        }
        if (packetType == MessageTypeHeader::BUNDLE) {
            BundleHeader bundleHeader;
            packet->RemoveHeader(bundleHeader);
            Bundle bundle;
            bundle.hop = bundleHeader.GetHop() + 1;
            bundle.id = bundleHeader.GetBundleId();
            bundle.source = bundleHeader.GetSourceIp();
            bundle.destination = bundleHeader.GetDestIp();
            bundle.creationTime = bundleHeader.GetCreationTime();
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << Ipv4Address(m_myIp)
                << " received bundle " << Ipv4Address(bundle.source) << "::" << bundle.id
                << " to " << Ipv4Address(bundle.destination)
            );
            if (bundle.destination == m_myIp) {
                NS_LOG_UNCOND("Bundle reached destination");
                continue;
            }

            RemoveExpiredBundles();
            if (m_buffer.size() >= m_maxBufferSize) {
                NS_LOG_UNCOND("Bundle dropped due to buffer overflow");
                continue; // there are no place for this bundle
            }
            NS_LOG_UNCOND("Bundle added to buffer");
            m_buffer.push_back(bundle);
            // RemoveOldBundle();

            Time jitter = GetJitter();
            Simulator::Schedule(jitter, &SimpleDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
            continue;
        }
        if (packetType == MessageTypeHeader::BUNDLE_ACK) {
            BundleAckHeader bundleAckHeader;
            packet->RemoveHeader(bundleAckHeader);
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << m_myIp
                << ": BUNDLE ACK from " << topHeader.GetNodeIP());
            RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
            if (m_nodeType == NODE_TYPE_FERRY) {
                if (nodeType[topHeader.GetNodeIP().Get()] == NODE_TYPE_GROUND)
                    ScheduleAcceptTransfer(topHeader.GetNodeIP());
            }
            if (m_nodeType == NODE_TYPE_GROUND) {
                ScheduleTransfer(topHeader.GetNodeIP());
            }
            continue;
        }
        if (packetType == MessageTypeHeader::FERRY_ACCEPT_TRANSFER) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << m_myIp
                << ": ACCEPT TRANSFER from ferry node " << topHeader.GetNodeIP()
            );
            ScheduleTransfer(topHeader.GetNodeIP());
        }
    }

}


void SimpleDtnApp::OnNeighborDiscovery(Ipv4Address neighborIp) {

}

bool SimpleDtnApp::ShouldForward(const Bundle& bundle, Ipv4Address neighborIp)
{
    if (neighborIp == bundle.source) return false;
    if (neighborIp == bundle.destination) return true;
    return true; // Epidemic
}

void SimpleDtnApp::GenerateBundle() {
    m_bundleIdCounter++;
    Bundle b;
    b.hop = 0;
    b.id = m_bundleIdCounter;
    b.source = m_myIp;
    b.creationTime = Simulator::Now().GetMicroSeconds();

    Ipv4Address dest = m_myIp;
    while (dest == m_myIp) { // so that destIP not equal my ip
        dest = groundNodeIps[m_rand->GetInteger(0, groundNodeIps.size() - 1)];
    }

    b.destination = dest;

    m_buffer.push_back(b);
    RemoveOldBundle();

    NS_LOG_UNCOND("Node " << m_myIp << ": CREATED Bundle " << b.id << " to " << b.destination);

    // generate based on poisson distribution
    double jitter = m_rand->GetValue(0.0, 1.0);
    double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    Simulator::Schedule(Seconds(IAT), &SimpleDtnApp::GenerateBundle, this);
}

void SimpleDtnApp::RemoveExpiredBundles() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();
    while (m_buffer.size() > 0 && m_buffer[0].creationTime + config.bundleTTL < currentTime) {
        NS_LOG_UNCOND("Node " << m_myIp << ": EXPIRED Bundle " << m_buffer[0].id);
        m_buffer.erase(m_buffer.begin());
    }
}

void SimpleDtnApp::RemoveOldBundle() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    while (m_buffer.size() > m_maxBufferSize) {
        NS_LOG_UNCOND("Node " << m_myIp << ": DROPPED Bundle " << m_buffer[0].id);
        m_buffer.erase(m_buffer.begin());
    }
}

void SimpleDtnApp::RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            m_buffer.erase(it);
            return;
        }
    }
}

void SimpleDtnApp::BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            it->flag_waitingAck = false; // set flag = false so bundle can be transfered again later
            return;
        }
    }
}



void ScheduleNextWaypoint(const std::vector<point2D>& points,
    const std::vector<uint32_t>& order,
    double speed,
    uint32_t index,
    Ptr<ConstantVelocityMobilityModel> mobility) {

    uint32_t nextIndex = (index + 1) % order.size();

    point2D relative = { points[order[nextIndex]].x - points[order[index]].x,
                         points[order[nextIndex]].y - points[order[index]].y };

    double distance = std::sqrt(relative.x * relative.x + relative.y * relative.y);
    double time = distance / speed;

    Simulator::Schedule(Seconds(time), &ScheduleNextWaypoint, points, order, speed, nextIndex, mobility);

    point2D velocity;
    velocity.x = relative.x / distance * speed;
    velocity.y = relative.y / distance * speed;

    mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));

}

void ScheduleTSPMobility(
    const std::vector<point2D>& points,
    const std::vector<uint32_t>& order,
    double speed,
    double height,
    Ptr<ConstantVelocityMobilityModel> mobility) {

    mobility->SetPosition(Vector(points[order[0]].x, points[order[0]].y, height));
    ScheduleNextWaypoint(points, order, speed, 0, mobility);
}

// ===========================================================================
// 3. MAIN SCRIPT (ĐÃ SỬA THÀNH OCB)
// ===========================================================================
int main(int argc, char* argv[])
{
    // set random seed so every run is the same
    srand(1337);
    SeedManager::SetSeed(1337);

    // initialize global random generator
    m_rand = CreateObject<UniformRandomVariable>();

    // parse cmd line args
    CommandLine cmd;
    cmd.AddValue("commRange", "Communication range", config.commRange);
    cmd.Parse(argc, argv);

    LogComponentEnable("SimpleDtnApp", LOG_LEVEL_INFO);

    NodeContainer allNodes;
    NodeContainer GroundNodes;
    GroundNodes.Create(config.nGrounds);
    NodeContainer ferryNode;
    ferryNode.Create(1);
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

    double areaPadding = 200; // padding to avoid node on edge of the map
    double rangePadding = config.commRange + 30; // padding tp avoid node near communication range
    std::vector<point2D> points =
        PoissonDisk_RandomSample(config.nGrounds, config.commRange + rangePadding, config.areaWidth - areaPadding);

    std::vector<uint32_t> order = TSPClassicGA(points);
    order = TSPTwoOptOptimize(points, order);

    std::vector<std::vector<uint32_t>> groundNodeClusters = KMeans(points, config.nFerrys);

    for (uint32_t i = 0; i < config.nGrounds; i++) {
        PointToPointHelper pointToPoint;
        pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
        pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));
        uint32_t index = order[i];
        uint32_t nextIndex = order[(i + 1) % config.nGrounds];
        NodeContainer nc;
        nc.Add(GroundNodes.Get(index));
        nc.Add(GroundNodes.Get(nextIndex));
        pointToPoint.Install(nc);
    }

    MobilityHelper mobility;
    // mobility.SetPositionAllocator("ns3::ListPositionAllocator");
    // mobility.Install(GroundNodes);

    Ptr<ListPositionAllocator> ground_lpa = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < config.nGrounds; i++) {
        points[i].x += areaPadding / 2;
        points[i].y += areaPadding / 2;
        ground_lpa->Add(Vector(points[i].x, points[i].y, 0));
    }
    mobility.SetPositionAllocator(ground_lpa);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(GroundNodes);

    // Cấu hình Ferry bay
    MobilityHelper ferryMobility;
    ferryMobility.SetPositionAllocator("ns3::ListPositionAllocator"); // Đặt vị trí ban đầu cụ thể
    Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
    lpa->Add(Vector(config.areaWidth / 2, config.areaWidth / 2, config.ferryHeight)); // Bắt đầu tại (0,0) cao 50m
    ferryMobility.SetPositionAllocator(lpa);

    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // Kích hoạt vận tốc cho Ferry (Nếu không nó sẽ đứng im)
    Ptr<ConstantVelocityMobilityModel> cvmm = ferryNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    ScheduleTSPMobility(points, order, config.ferrySpeed, config.ferryHeight, cvmm);

    // ==========================================================
    // INTERNET STACK & APP
    // ==========================================================

    InternetStackHelper internet;
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    for (uint32_t n = 0; n < config.nGrounds; n++) {
        groundNodeIps.push_back(i.GetAddress(n));
    }

    // Cài App cho Ground Nodes
    for (uint32_t n = 0; n < config.nGrounds; n++)
    {
        Ptr<Socket> socket = Socket::CreateSocket(GroundNodes.Get(n), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(n), config.groundBufferSize, NODE_TYPE_GROUND);
        GroundNodes.Get(n)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime));
        if (n == 0) // TODO temp 
            app->EnableBundleGeneration(10.0);
        nodeType[i.GetAddress(n).Get()] = NODE_TYPE_GROUND;
    }

    // Cài App cho Ferry (Index trong container i là config.nGrounds)
    {
        Ptr<Socket> socket = Socket::CreateSocket(ferryNode.Get(0), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(config.nGrounds), config.ferryBufferSize, NODE_TYPE_FERRY); // Buffer lớn cho Ferry
        ferryNode.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime));
        nodeType[i.GetAddress(config.nGrounds).Get()] = NODE_TYPE_FERRY;
    }

    // Animation
    AnimationInterface anim("trace/dtn-adhoc-anim.xml");
    // uint32_t bgImg = anim.AddResource("trace/flood.png");
    double bgScaleX = config.areaWidth / 1000.0;
    double bgScaleY = config.areaWidth / 964.0;
    if (config.enable_background)
        anim.SetBackgroundImage("/home/minh/ns-allinone-3.30.1/ns-3.30.1/trace/flood2.png", 0, 0, bgScaleX, bgScaleY, 0.5);

    // anim.SetMaxPktsPerTraceFile(500000);
    for (uint32_t n = 0; n < config.nGrounds; n++) {
        // anim.UpdateNodeColor(GroundNodes.Get(n)->GetId(), 0, 255, 0);
        anim.UpdateNodeSize(GroundNodes.Get(n)->GetId(), 80, 80);
    }
    { // set color for each cluster
        int colorIndex = 0;
        for (auto& cluster : groundNodeClusters) {
            for (auto& node : cluster) {

                anim.UpdateNodeColor(GroundNodes.Get(node)->GetId(), colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
                anim.UpdateNodeDescription(GroundNodes.Get(node)->GetId(), std::to_string(colorIndex));
            }
            colorIndex++;
        }
    }
    anim.UpdateNodeColor(ferryNode.Get(0)->GetId(), 255, 0, 0);
    anim.UpdateNodeSize(ferryNode.Get(0)->GetId(), 60, 60);

    FerryVisualizer::SetUp();
    Simulator::Stop(Seconds(config.simTime)); // Đã set trong app
    Simulator::Run();
    Simulator::Destroy();
    FerryVisualizer::CleanUp();

    return 0;
}