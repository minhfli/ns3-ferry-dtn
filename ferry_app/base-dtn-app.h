#ifndef BASE_DTN_APP_H
#define BASE_DTN_APP_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/wave-module.h" 
#include "ns3/point-to-point-module.h"

#include "../ferry_helper/global.h"
#include "../ferry_helper/config.h"
#include "../ferry_helper/report.h"

#include "../ferry_helper/ferry-helper.h"
#include "../ferry_helper/tsp-helper.h"
#include "../ferry_helper/cluster-helper.h"
#include "../ferry_helper/packet-helper.h"
#include "../ferry_helper/viz-helper.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
using namespace ns3;

Time GetJitter() {
    return MicroSeconds(m_rand->GetInteger(config.jitterAmount, config.jitterAmount * 3));
}

class BaseDtnApp : public Application {
    public:
    BaseDtnApp();
    ~BaseDtnApp();

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::BaseDtnApp")
            .SetParent<Application>();
            // .AddConstructor<BaseDtnApp>(); // cannot add constructor since this is abstract class
        return tid;
    }

    // Setup chung cho mọi App
    void Setup(Ptr<Node> node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType);
    void EnableBundleGeneration(double rate, bool inversed = true);
    void SetGroup(uint8_t groupId) {
        m_groupId = groupId;
    }

    /**
     * Setup mobility cho node
     * @param servingNodes Các ground node mà ferry phải phục vụ
     * @param points Vị trí của các node đó
     */
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) = 0;

    protected:
    void StartApplication(void) override;
    void StopApplication(void) override;

    // Logic nhận gói tin sẽ khác nhau hoàn toàn -> Pure Virtual Function
    virtual void ReceivePacket(Ptr<Socket> socket) = 0;
    // Lấy route của ferry, dưới dạng node IP
    virtual std::vector<uint32_t> GetServingNodeRoute() = 0;
    // Lấy route của ferry, dưới dạng waypoint
    virtual std::vector<point2D> GetServingWaypointRoute() = 0;

    // --- Các hàm tiện ích dùng chung (Sending functions) ---
    void Beacon();
    void SendGroundHello(Ipv4Address ferryIp);
    void SendFerryHello(Ipv4Address ferryIp);
    void SendFerryAcceptTransfer(Ipv4Address groundIp);
    void SendBundle(Bundle& bundle, Ipv4Address neighborIp);
    void SendBundleAck(Bundle bundle, Ipv4Address neighborIp);
    void Schedule_FerryToGround_Transfer(Ipv4Address groundIp);
    void Schedule_GroundToFerry_Transfer(Ipv4Address ferryIp);

    // --- Bundle Logic dùng chung ---
    void GenerateBundle();
    void RemoveExpiredBundles();
    void RemoveOldBundle();
    void RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp);
    void BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp);

    // --- Variables (Chuyển từ private sang protected) ---
    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    uint8_t m_groupId; // for clustering
    uint8_t m_mode; // curent operation mode
    bool m_enableFerryComm; // enable communication between ferry 

    double m_bundleGenRate;
    std::vector<Bundle> m_buffer;
    uint32_t m_maxBufferSize;
    uint32_t m_bundleIdCounter;

    // Mobility common vars
    Ptr<ConstantVelocityMobilityModel> m_mobility;
};

NS_OBJECT_ENSURE_REGISTERED(BaseDtnApp);

BaseDtnApp::BaseDtnApp()
    : m_port(80), m_maxBufferSize(100), m_bundleIdCounter(0) {
}

BaseDtnApp::~BaseDtnApp() {}

void BaseDtnApp::Setup(Ptr<Node> node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType) {
    m_node = node;
    m_socket = socket;
    m_myIp = myIp;
    m_maxBufferSize = bufferSize;
    m_nodeType = nodeType;
    m_bundleGenRate = 0.0;

    m_buffer.clear();

    m_mobility = m_node->GetObject<ConstantVelocityMobilityModel>();
    NS_LOG_UNCOND(nodeId[m_myIp.Get()]);
    NS_ASSERT(m_mobility != nullptr);
}

void BaseDtnApp::EnableBundleGeneration(double rate, bool inversed) {
    if (inversed) {
        m_bundleGenRate = 1.0 / rate;
    }
    else {
        m_bundleGenRate = rate;
    }
    double averageBundleTime = 1.0 / m_bundleGenRate;
    double startTime = 0.1 + m_rand->GetValue(0.0, averageBundleTime);
    Simulator::Schedule(Seconds(startTime + config.startGeneraionTime), &BaseDtnApp::GenerateBundle, this);
}

void BaseDtnApp::StartApplication(void) {
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port)) == -1) {
        NS_LOG_UNCOND("Bind failed for node " << nodeId[m_myIp.Get()]);
        NS_ASSERT(false);
        return;
    }
    m_socket->SetRecvCallback(MakeCallback(&BaseDtnApp::ReceivePacket, this));
    m_socket->SetAllowBroadcast(true);
    NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " started listening on port " << m_port);

    if (m_nodeType == 0) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a GROUND node.");
    }
    else {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a FERRY node.");
        Simulator::Schedule(GetJitter(), &BaseDtnApp::Beacon, this);
    }
}

void BaseDtnApp::StopApplication(void) {
    if (m_socket) { m_socket->Close(); }
}

void BaseDtnApp::Beacon() {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_BEACON);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, broadcast);

    FerryVisualizer::logBeacon(nodeId[m_myIp.Get()]);

    Time nextBeacon = Seconds(m_rand->GetValue(0.0, config.beaconRandomness)) + Seconds(config.beaconInterval);
    // Schedule next Hello
    Simulator::Schedule(nextBeacon, &BaseDtnApp::Beacon, this);
}

void BaseDtnApp::SendGroundHello(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::GROUND_HELLO);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendFerryHello(Ipv4Address ferryIp) {
    // TODO
}

void BaseDtnApp::SendFerryAcceptTransfer(Ipv4Address groundIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_ACCEPT_TRANSFER);
    header.SetNodeIP(m_myIp);

    FerryRouteHeader fRouteHeader; // TODO handling waypoints
    fRouteHeader.SetGroup(m_groupId);
    fRouteHeader.SetCount(0);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(groundIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendBundle(Bundle& bundle, Ipv4Address neighborIp) {
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
        &BaseDtnApp::BundleAckTimeout, this, bundle.id, bundle.source.Get());

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendBundleAck(Bundle bundle, Ipv4Address neighborIp) {
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

void BaseDtnApp::Schedule_FerryToGround_Transfer(Ipv4Address groundIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false && bundle.destination == groundIp) {
            Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, bundle, groundIp);
            return;
        }
    }
    RemoveExpiredBundles();
    if (m_buffer.size() >= m_maxBufferSize) {
        return; // there are no buffer left to receive additional bundle
    }
    Simulator::Schedule(jitter, &BaseDtnApp::SendFerryAcceptTransfer, this, groundIp);
}

void BaseDtnApp::Schedule_GroundToFerry_Transfer(Ipv4Address ferryIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    if (m_groupId != nodeGroup[ferryIp.Get()]) {
        // send bundle that belongs to ground node with ferry group id
        for (auto& bundle : m_buffer) {
            if (bundle.flag_waitingAck == false && nodeGroup[bundle.destination.Get()] == nodeGroup[ferryIp.Get()]) {
                Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, bundle, ferryIp);
                return;
            }
        }
        return; // not in the same group
    }
    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, bundle, ferryIp);
            return;
        }
    }
}


void BaseDtnApp::GenerateBundle() {
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
    Report::bundleCount++;

    RemoveOldBundle();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": CREATED Bundle " << b.id << " to " << b.destination);

    // generate based on poisson distribution
    double jitter = m_rand->GetValue(0.0, 1.0);
    double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    Simulator::Schedule(Seconds(IAT), &BaseDtnApp::GenerateBundle, this);
}

void BaseDtnApp::RemoveExpiredBundles() {
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();

    m_buffer.erase(std::remove_if(m_buffer.begin(), m_buffer.end(), [&](const Bundle& b) {
        // Điều kiện 1: Hết TTL
        if (b.creationTime + config.bundleTTL < currentTime) return true;

        // Điều kiện 2: Impossible (nếu bay thẳng vẫn không kịp)
        Vector3D currentPos = m_mobility->GetPosition();
        point2D target = groundNodePos[rawNodeId(b.destination.Get())];
        double dist = point2D{ target.x - currentPos.x, target.y - currentPos.y }.length() - 2.0 * config.commRange;

        double timeToReach = dist / config.ferrySpeed;
        return (Simulator::Now() + Seconds(timeToReach) > MicroSeconds(b.creationTime + config.bundleTTL));
        }), m_buffer.end());

}

void BaseDtnApp::RemoveOldBundle() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    while (m_buffer.size() > m_maxBufferSize) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": DROPPED Bundle " << m_buffer[0].id);
        m_buffer.erase(m_buffer.begin());
    }
}

void BaseDtnApp::RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            m_buffer.erase(it);
            return;
        }
    }
}

void BaseDtnApp::BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            it->flag_waitingAck = false; // set flag = false so bundle can be transfered again later
            return;
        }
    }
}

#endif