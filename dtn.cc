/*
 * DTN Base Framework for NS-3
 * Focus: Store-Carry-Forward Logic (The ONE style)
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/pyviz.h"

#include <vector>
#include <algorithm>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleDtnApp");

// ===========================================================================
// 1. DTN HEADER: Định nghĩa cấu trúc gói tin (Thay thế việc xử lý chuỗi)
// ===========================================================================
class DtnHeader : public Header
{

    private:
    uint8_t m_type;
    uint32_t m_bundleId;
    uint32_t m_sourceIp; // Lưu dưới dạng uint32 để serialize
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
      // Type(1) + BundleId(4) + Src(4) + Dst(4)
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
// 2. BUNDLE STORAGE: Quản lý bộ đệm gói tin
// ===========================================================================
struct Bundle {
    uint32_t id;
    Ipv4Address source;
    Ipv4Address destination;
    uint32_t size;
    // Có thể thêm: TTL, CreationTime, PayloadContent...
};

class SimpleDtnApp : public Application
{
    public:
    SimpleDtnApp();
    virtual ~SimpleDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize);

    // Hàm để người dùng tạo bundle mới từ kịch bản simulation
    void GenerateBundle(Ipv4Address dest, uint32_t size);

    private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void ReceivePacket(Ptr<Socket> socket);
    void SendHello();

    // --- CORE LOGIC: Nơi bạn sẽ cài thuật toán Routing ---
    void OnNeighborDiscovery(Ipv4Address neighborIp);
    bool ShouldForward(const Bundle& bundle, Ipv4Address neighborIp);
    // -----------------------------------------------------

    void SendBundle(Bundle bundle, Ipv4Address neighborIp);

    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    EventId m_helloEvent;

    std::vector<Bundle> m_buffer; // Store
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
    : m_port(80), m_maxBufferSize(100), m_bundleIdCounter(0)
{
}

SimpleDtnApp::~SimpleDtnApp()
{
}

void SimpleDtnApp::Setup(Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize)
{
    m_socket = socket;
    m_myIp = myIp;
    m_maxBufferSize = bufferSize;
}

void SimpleDtnApp::StartApplication(void)
{
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port)) == -1)
    {
        NS_LOG_UNCOND("Bind failed for node " << m_myIp);
        return;
    }
    m_socket->SetRecvCallback(MakeCallback(&SimpleDtnApp::ReceivePacket, this));
    m_socket->SetAllowBroadcast(true);

    NS_LOG_UNCOND("Node " << m_myIp << " started listening on port " << m_port);

    // Bắt đầu gửi Hello định kỳ
    // SendHello();
    Simulator::Schedule(Seconds(0), &SimpleDtnApp::SendHello, this);
    // Ví dụ: Node
    // Simulator::Schedule(Seconds(1.0), &SimpleDtnApp::SendHello, this);
}

void SimpleDtnApp::StopApplication(void)
{
    if (m_socket)
    {
        m_socket->Close();
    }
    Simulator::Cancel(m_helloEvent);
}

// ---------------------------------------------------------------------------
// GỬI HELLO (BEACON)
// ---------------------------------------------------------------------------
void SimpleDtnApp::SendHello()
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Sending HELLO");
    DtnHeader header;
    header.SetType(DtnHeader::HELLO);
    header.SetSourceIp(m_myIp);

    Ptr<Packet> packet = Create<Packet>(10); // Payload nhỏ
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);

    // Dùng SendTo thay vì Connect + Send cho broadcast
    m_socket->SendTo(packet, 0, broadcast);

    // Lên lịch gửi Hello tiếp theo sau 10 giây
    m_helloEvent = Simulator::Schedule(Seconds(10.0), &SimpleDtnApp::SendHello, this);
}

// ---------------------------------------------------------------------------
// NHẬN GÓI TIN
// ---------------------------------------------------------------------------
void SimpleDtnApp::ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Received packet");
    while ((packet = socket->RecvFrom(from)))
    {
        DtnHeader header;
        packet->RemoveHeader(header); // Bóc tách Header

        if (header.GetType() == DtnHeader::HELLO)
        {
          // Phát hiện hàng xóm! Kích hoạt logic định tuyến
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Met neighbor " << header.GetSourceIp());
            OnNeighborDiscovery(header.GetSourceIp());
        }
        else if (header.GetType() == DtnHeader::BUNDLE)
        {
          // Nhận được gói dữ liệu
            Ipv4Address finalDest = header.GetDestIp();

            if (finalDest == m_myIp)
            {
                NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": >>> DELIVERED Bundle " << header.GetBundleId());
            }
            else
            {
              // Store: Lưu vào buffer để chuyển tiếp sau
              // Kiểm tra xem đã có gói này chưa để tránh loop
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

                    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": Stored Bundle " << header.GetBundleId() << " (Hop count ++)");
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// LOGIC ĐỊNH TUYẾN (Nơi bạn cần sửa cho PROPHET)
// ---------------------------------------------------------------------------
void SimpleDtnApp::OnNeighborDiscovery(Ipv4Address neighborIp)
{
  // Duyệt qua buffer hiện tại
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

  // --- BASE LOGIC: EPIDEMIC (Gặp ai cũng gửi, trừ nguồn) ---
    if (neighborIp == bundle.source) return false;

    // Nếu hàng xóm là đích -> Gửi ngay
    if (neighborIp == bundle.destination) return true;

    // --- LOGIC PROPHET SẼ CHÈN VÀO ĐÂY ---
    // Ví dụ: if (Predictability[neighborIp][bundle.destination] > Predictability[m_myIp][bundle.destination]) return true;

    return true; // Mặc định Epidemic: luôn gửi
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
    m_socket->SetAllowBroadcast(false);
    m_socket->SendTo(packet, 0, dest);
}

// Hàm khởi tạo gói tin từ script
void SimpleDtnApp::GenerateBundle(Ipv4Address dest, uint32_t size)
{
    m_bundleIdCounter++;
    Bundle b;
    b.id = m_bundleIdCounter; // Sinh ID đơn giản, thực tế nên kết hợp NodeID
    b.source = m_myIp;
    b.destination = dest;
    b.size = size;

    m_buffer.push_back(b);
    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << m_myIp << ": CREATED Bundle " << b.id << " for " << dest);
}


// ===========================================================================
// 3. MAIN SCRIPT
// ===========================================================================
int main(int argc, char* argv[])
{
    float communicationRange = 1500000.0; // in meters

    CommandLine cmd;
    cmd.AddValue("communicationRange", "Communication range of each node in meters", communicationRange);
    cmd.Parse(argc, argv);
    // Bật log để xem kết quả
    LogComponentEnable("SimpleDtnApp", LOG_LEVEL_INFO);
    std::string phyMode("DsssRate1Mbps");
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                       StringValue(phyMode));

    uint32_t nNodes = 3;
    double simulationTime = 200.0;

    NodeContainer allNodes;
    NodeContainer GroundNodes;
    GroundNodes.Create(nNodes);
    NodeContainer ferryNode;
    ferryNode.Create(1);
    allNodes.Add(GroundNodes);
    allNodes.Add(ferryNode);

    // 1. Cấu hình Wifi Adhoc
    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue(phyMode),
                                 "ControlMode", StringValue(phyMode)); // enable RTS/CTS for all packets except Broadcast

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel;
    // Bỏ qua việc mô phỏng nhiễu để tập trung vào logic DTN
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(communicationRange));
    // set error model = 0

    wifiPhy.SetChannel(wifiChannel.Create());


    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");



    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, allNodes);

    // 2. Cấu hình Mobility (Random Walk để các node gặp nhau)
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                   "X", StringValue("200.0"),
                                   "Y", StringValue("200.0"),
                                   "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=150]")); // Bán kính 300m
    // mobility.SetMobilityModel("ns3::StationaryMobilityModel");
    mobility.Install(GroundNodes);

    MobilityHelper ferryMobility;
    ferryMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                       "MinX", DoubleValue(0.0),
                                       "MinY", DoubleValue(0.0),
                                       "DeltaX", DoubleValue(400.0),
                                       "DeltaY", DoubleValue(400.0),
                                       "GridWidth", UintegerValue(5),
                                       "LayoutType", StringValue("RowFirst"));
    ferryMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ferryMobility.Install(ferryNode);

    // 3. Cài đặt Internet Stack
    InternetStackHelper internet;
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    // 4. Cài đặt DTN APP lên TẤT CẢ các node
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    for (uint32_t n = 0; n < nNodes; n++)
    {
        Ptr<Socket> socket = Socket::CreateSocket(GroundNodes.Get(n), tid);
        Ptr<SimpleDtnApp> app = CreateObject<SimpleDtnApp>();
        app->Setup(socket, i.GetAddress(n), 50); // Buffer size 50
        GroundNodes.Get(n)->AddApplication(app);
        app->SetStartTime(Seconds(1.0 + n)); // Giãn thời gian start để tránh xung đột
        app->SetStopTime(Seconds(simulationTime));
    }

    // Animation (Optional)
    AnimationInterface anim("trace/dtn-anim.xml");

    RngSeedManager::SetSeed(3); // Hoặc time(NULL)
    RngSeedManager::SetRun(7);
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
