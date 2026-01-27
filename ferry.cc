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


#include "ferry_helper/ferry-helper.h"
#include "ferry_helper/tsp-helper.h"
#include "ferry_helper/cluster-helper.h"

#include <vector>
#include <algorithm>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleDtnApp");

constexpr uint8_t NODE_TYPE_GROUND = 0;
constexpr uint8_t NODE_TYPE_FERRY = 1;

// ===========================================================================
// Global variables
// ===========================================================================
struct Config {
    // sim config
    double simTime = 3000.0; // seconds
    double commRange = 150.0; // meters
    double areaWidth = 4000; // meters
    uint32_t nGrounds = 50;
    uint32_t nFerrys = 10;

    // node config
    double beaconInterval = 5.0;    // seconds
    double beaconRandomness = 3.0; // seconds
    uint32_t jitterAmount = 20; // milisecconds, used for exchange message

    double ferrySpeed = 15.0;      // m/s
    uint16_t dtnPort = 9000;
    uint32_t groundBufferSize = 50; // bundles
    uint32_t ferryBufferSize = 200;  // bundles

    // physical payload size of a bundle
    uint32_t bundlePayloadSize = 102400; // 100Kb 

} config;

Ptr<UniformRandomVariable> m_rand;

std::vector<Ipv4Address> groundNodeIps;

// ===========================================================================
// 1. DTN HEADER (Giữ nguyên)
// ===========================================================================
class DtnHeader : public Header
{
    private:
    uint8_t m_type;
    uint32_t m_bundleId;
    uint32_t m_sourceIp;
    uint32_t m_destIp;

    public:
    enum MessageType {
        FERRY_BEACON = 1, // ferry broadcast on interval
        FERRY_HELLO = 2, // ferry unicast when receive beacon
        FERRY_ACCEPT_TRANSFER = 3, // ferry send after tranfering all of the bundle it need to send to the ground node 
        // FERRY_HELLO_ACK = 3, // ferry unicast when receive hello2

        GROUND_HELLO = 11, // ground node send when beaconed by ferry
        // GROUND_HELLO_ACK = 4, // temp, not used

        BUNDLE = 25, // bundle packet
        BUNDLE_ACK = 26, // bundle ack, if a node receive this, the bundle is deleted from its buffer
    };

    DtnHeader() : m_type(FERRY_HELLO), m_bundleId(0), m_sourceIp(0), m_destIp(0) {}

    void SetType(uint8_t type) { m_type = type; }
    uint8_t GetType() const { return m_type; }

    void SetBundleId(uint32_t id) { m_bundleId = id; }
    uint32_t GetBundleId() const { return m_bundleId; }

    void SetSourceIp(Ipv4Address ip) { m_sourceIp = ip.Get(); }
    Ipv4Address GetSourceIp() const { return Ipv4Address(m_sourceIp); }

    void SetDestIp(Ipv4Address ip) { m_destIp = ip.Get(); }
    Ipv4Address GetDestIp() const { return Ipv4Address(m_destIp); }

    static TypeId GetTypeId(void)
    {
        static TypeId tid = TypeId("ns3::DtnHeader")
            .SetParent<Header>()
            .AddConstructor<DtnHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Type=" << (int)m_type << " Bid=" << m_bundleId;
    }

    virtual uint32_t GetSerializedSize(void) const
    {
        return 1 + 4 + 4 + 4;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteU8(m_type);
        start.WriteHtonU32(m_bundleId);
        start.WriteHtonU32(m_sourceIp);
        start.WriteHtonU32(m_destIp);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_type = start.ReadU8();
        m_bundleId = start.ReadNtohU32();
        m_sourceIp = start.ReadNtohU32();
        m_destIp = start.ReadNtohU32();
        return GetSerializedSize();
    }
};

// ===========================================================================
// 2. APP & BUNDLE STORAGE (Có cập nhật Jitter)
// ===========================================================================
struct Bundle {
    uint32_t id;
    Ipv4Address source;
    Ipv4Address destination;
};

class SimpleDtnApp : public Application
{
    public:
    SimpleDtnApp();
    virtual ~SimpleDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType = 0);
    void GenerateBundle();
    void EnableBundleGeneration(double rate, bool inversed = true);

    private:
    static Time GetJitter();

    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void ReceivePacket(Ptr<Socket> socket);
    void Beacon();

    void OnNeighborDiscovery(Ipv4Address neighborIp);
    bool ShouldForward(const Bundle& bundle, Ipv4Address neighborIp);
    void SendBundle(Bundle bundle, Ipv4Address neighborIp);

    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    EventId m_helloEvent;

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
    // double averageBundleTime = 1.0 / m_bundleGenRate;
    // Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    // double startTime = rand->GetValue(0.0, averageBundleTime);
    // Simulator::Schedule(Seconds(startTime), &SimpleDtnApp::GenerateBundle, this);
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
    Simulator::Cancel(m_helloEvent);
}

Time SimpleDtnApp::GetJitter() {
    return MilliSeconds(m_rand->GetInteger(1, config.jitterAmount));
}

void SimpleDtnApp::Beacon() {
    // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Sending HELLO (OCB)");
    DtnHeader header;
    header.SetType(DtnHeader::FERRY_BEACON);
    header.SetSourceIp(m_myIp);

    Ptr<Packet> packet = Create<Packet>(100); // Tăng size lên 1 chút giả lập metadata
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, broadcast);

    Time nextBeacon = Seconds(m_rand->GetValue(0.0, config.beaconRandomness)) + Seconds(config.beaconInterval);
    // Schedule next Hello
    m_helloEvent = Simulator::Schedule(nextBeacon, &SimpleDtnApp::Beacon, this);
}

void SimpleDtnApp::ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        DtnHeader header;
        packet->RemoveHeader(header);

        // Tự mình gửi thì bỏ qua (Loopback)
        if (header.GetSourceIp() == m_myIp) continue;

        if (header.GetType() == DtnHeader::FERRY_BEACON)
        {
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": BEACON from Ferry " << header.GetSourceIp());
            OnNeighborDiscovery(header.GetSourceIp());
        }
        else if (header.GetType() == DtnHeader::BUNDLE)
        {
            Ipv4Address finalDest = header.GetDestIp();
            if (finalDest == m_myIp)
            {
                NS_LOG_UNCOND(">>> Node " << m_myIp << " RECEIVED Bundle " << header.GetBundleId());
            }
            else
            {
                // Logic Store check
                bool exists = false;
                for (const auto& b : m_buffer) if (b.id == header.GetBundleId()) exists = true;

                if (!exists && m_buffer.size() < m_maxBufferSize)
                {
                    Bundle newBundle;
                    newBundle.id = header.GetBundleId();
                    newBundle.source = header.GetSourceIp();
                    newBundle.destination = header.GetDestIp();
                    // newBundle.size = packet->GetSize();
                    m_buffer.push_back(newBundle);
                    NS_LOG_UNCOND("Node " << m_myIp << ": Stored Bundle " << header.GetBundleId());
                }
            }
        }
    }
}

void SimpleDtnApp::OnNeighborDiscovery(Ipv4Address neighborIp)
{
    for (const auto& bundle : m_buffer)
    {
        if (ShouldForward(bundle, neighborIp))
        {
            SendBundle(bundle, neighborIp);
        }
    }
}

bool SimpleDtnApp::ShouldForward(const Bundle& bundle, Ipv4Address neighborIp)
{
    if (neighborIp == bundle.source) return false;
    if (neighborIp == bundle.destination) return true;
    return true; // Epidemic
}

void SimpleDtnApp::SendBundle(Bundle bundle, Ipv4Address neighborIp)
{
    DtnHeader header;
    header.SetType(DtnHeader::BUNDLE);
    header.SetBundleId(bundle.id);
    header.SetSourceIp(bundle.source);
    header.SetDestIp(bundle.destination);

    Ptr<Packet> packet = Create<Packet>(config.bundlePayloadSize);
    packet->AddHeader(header);

    InetSocketAddress dest = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, dest);
}

void SimpleDtnApp::GenerateBundle() {
    m_bundleIdCounter++;
    Bundle b;
    b.id = m_bundleIdCounter;
    b.source = m_myIp;

    Ipv4Address dest = groundNodeIps[rand() % groundNodeIps.size()];
    b.destination = dest;

    m_buffer.push_back(b);
    NS_LOG_UNCOND("Node " << m_myIp << ": CREATED Bundle " << b.id);

    // generate based on poisson distribution
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    double jitter = rand->GetValue(0.0, 1.0);
    double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    Simulator::Schedule(Seconds(IAT), &SimpleDtnApp::GenerateBundle, this);
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
    lpa->Add(Vector(config.areaWidth / 2, config.areaWidth / 2, 50)); // Bắt đầu tại (0,0) cao 50m
    ferryMobility.SetPositionAllocator(lpa);

    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // Kích hoạt vận tốc cho Ferry (Nếu không nó sẽ đứng im)
    Ptr<ConstantVelocityMobilityModel> cvmm = ferryNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    ScheduleTSPMobility(points, order, config.ferrySpeed, 50, cvmm);

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
        app->Setup(socket, i.GetAddress(n), 50, NODE_TYPE_GROUND);
        GroundNodes.Get(n)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime));
        app->EnableBundleGeneration(10.0);
    }

    // Cài App cho Ferry (Index trong container i là config.nGrounds)
    {
        Ptr<Socket> socket = Socket::CreateSocket(ferryNode.Get(0), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(config.nGrounds), 500, NODE_TYPE_FERRY); // Buffer lớn cho Ferry
        ferryNode.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(config.simTime));
    }

    // Animation
    AnimationInterface anim("trace/dtn-adhoc-anim.xml");
    // uint32_t bgImg = anim.AddResource("trace/flood.png");
    double bgScaleX = 4000.0 / 1000.0;
    double bgScaleY = 4000.0 / 964.0;
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

    Simulator::Stop(Seconds(config.simTime)); // Đã set trong app
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}