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
// *** QUAN TRỌNG: Thêm module wave cho OCB ***
#include "ns3/wave-module.h" 

#include <vector>
#include <algorithm>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleDtnApp");

constexpr uint8_t NODE_TYPE_GROUND = 0;
constexpr uint8_t NODE_TYPE_FERRY = 1;

// ===========================================================================
struct Config {
    // sim config
    double simTime = 300.0; // seconds
    double commRange = 100.0; // meters
    double areaWidth = 500; // meters

    // node config
    double helloInterval = 5.0;    // seconds
    double ferrySpeed = 15.0;      // m/s
    uint16_t dtnPort = 9000;
    uint32_t groundBufferSize = 50; // bundles
    uint32_t ferryBufferSize = 200;  // bundles

} config;


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
        HELLO = 1,
        BUNDLE = 2
    };

    DtnHeader() : m_type(HELLO), m_bundleId(0), m_sourceIp(0), m_destIp(0) {}

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
    uint32_t size;
};

class SimpleDtnApp : public Application
{
    public:
    SimpleDtnApp();
    virtual ~SimpleDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType = 0);
    void GenerateBundle(Ipv4Address dest, uint32_t size);
    void EnableBundleGeneration(double rate, bool inversed = true);

    private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void ReceivePacket(Ptr<Socket> socket);
    void SendHello();

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
        Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
        double jitter = rand->GetValue(0.0, 1.0); // Random từ 0 đến 1 giây

        Simulator::Schedule(Seconds(jitter), &SimpleDtnApp::SendHello, this);
    }
}

void SimpleDtnApp::StopApplication(void) {
    if (m_socket) { m_socket->Close(); }
    Simulator::Cancel(m_helloEvent);
}

void SimpleDtnApp::SendHello() {
    // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Sending HELLO (OCB)");
    DtnHeader header;
    header.SetType(DtnHeader::HELLO);
    header.SetSourceIp(m_myIp);

    Ptr<Packet> packet = Create<Packet>(100); // Tăng size lên 1 chút giả lập metadata
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, broadcast);

    // Schedule next Hello
    m_helloEvent = Simulator::Schedule(Seconds(2.0), &SimpleDtnApp::SendHello, this);
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

        if (header.GetType() == DtnHeader::HELLO)
        {
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": OCB HELLO from " << header.GetSourceIp());
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
                    newBundle.size = packet->GetSize();
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

    Ptr<Packet> packet = Create<Packet>(bundle.size);
    packet->AddHeader(header);

    InetSocketAddress dest = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, dest);
}

void SimpleDtnApp::GenerateBundle(Ipv4Address dest, uint32_t size)
{
    m_bundleIdCounter++;
    Bundle b;
    b.id = m_bundleIdCounter;
    b.source = m_myIp;
    b.destination = dest;
    b.size = size;

    m_buffer.push_back(b);
    NS_LOG_UNCOND("Node " << m_myIp << ": CREATED Bundle " << b.id);

    // generate based on poisson distribution
    // Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    // double jitter = rand->GetValue(0.0, 1.0);
    // double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    // Simulator::Schedule(Seconds(IAT), &SimpleDtnApp::GenerateBundle, this, dest, size);
}


// ===========================================================================
// 3. MAIN SCRIPT (ĐÃ SỬA THÀNH OCB)
// ===========================================================================
int main(int argc, char* argv[])
{
    // Tầm phủ sóng của 802.11p ~300-1000m tùy môi trường
    float communicationRange = 500.0;

    CommandLine cmd;
    cmd.AddValue("communicationRange", "Communication range", communicationRange);
    cmd.Parse(argc, argv);

    LogComponentEnable("SimpleDtnApp", LOG_LEVEL_INFO);


    uint32_t nNodes = 5;
    double simulationTime = 100.0;

    NodeContainer allNodes;
    NodeContainer GroundNodes;
    GroundNodes.Create(nNodes);
    NodeContainer ferryNode;
    ferryNode.Create(1);
    allNodes.Add(GroundNodes);
    allNodes.Add(ferryNode);

    // ==========================================================
    // CẤU HÌNH WIFI OCB (802.11p)
    // ==========================================================

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel;

    // Dùng mô hình suy hao Friis cơ bản
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(communicationRange));
    wifiPhy.SetChannel(wifiChannel.Create());


    // --- Thay thế WifiHelper bằng Wifi80211pHelper ---
    Wifi80211pHelper waveHelper = Wifi80211pHelper::Default();

    // Cấu hình Manager: OCB không có Association nên ConstantRate hoặc Minstrel đều được
    // Chuẩn 802.11p dùng kênh 10MHz. 6Mbps là tốc độ cơ sở tin cậy nhất.
    waveHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                       "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                       "NonUnicastMode", StringValue("OfdmRate6MbpsBW10MHz"));
    NqosWaveMacHelper ocbMAC = NqosWaveMacHelper::Default();
      // wifiMac.SetType("ns3::OcbWifiMac"); // Cấu hình MAC OCB

    NetDeviceContainer devices = waveHelper.Install(wifiPhy, ocbMAC, allNodes);

    // ==========================================================
    // MOBILITY
    // ==========================================================
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                   "X", DoubleValue(config.areaWidth / 2.0),
                                   "Y", DoubleValue(config.areaWidth / 2.0),
                                   "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=200]"));
    mobility.Install(GroundNodes);

    // Cấu hình Ferry bay
    MobilityHelper ferryMobility;
    ferryMobility.SetPositionAllocator("ns3::ListPositionAllocator"); // Đặt vị trí ban đầu cụ thể
    Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
    lpa->Add(Vector(0, 0, 50)); // Bắt đầu tại (0,0) cao 50m
    ferryMobility.SetPositionAllocator(lpa);

    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // Kích hoạt vận tốc cho Ferry (Nếu không nó sẽ đứng im)
    Ptr<ConstantVelocityMobilityModel> cvmm = ferryNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    cvmm->SetVelocity(Vector(20.0, 5.0, 0.0)); // Bay chéo với tốc độ ~20m/s

    // ==========================================================
    // INTERNET STACK & APP
    // ==========================================================
    InternetStackHelper internet;
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    // Cài App cho Ground Nodes
    for (uint32_t n = 0; n < nNodes; n++)
    {
        Ptr<Socket> socket = Socket::CreateSocket(GroundNodes.Get(n), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(n), 50, NODE_TYPE_GROUND);
        GroundNodes.Get(n)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(simulationTime));
    }

    // Cài App cho Ferry (Index trong container i là nNodes)
    {
        Ptr<Socket> socket = Socket::CreateSocket(ferryNode.Get(0), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(nNodes), 500, NODE_TYPE_FERRY); // Buffer lớn cho Ferry
        ferryNode.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(simulationTime));

        // Ferry tạo thử 1 gói tin gửi cho Node 0
        // Simulator::Schedule(Seconds(3.0), &SimpleDtnApp::GenerateBundle, app, i.GetAddress(0), 1024);
    }

    // Animation
    AnimationInterface anim("trace/dtn-ocb-anim.xml");
    // anim.SetMaxPktsPerTraceFile(500000);

    // Simulator::Stop(Seconds(simulationTime)); // Đã set trong app
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}