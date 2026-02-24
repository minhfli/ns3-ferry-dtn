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
#pragma region Structs

struct NeighborInfomation {
    uint32_t ip;
    std::vector<uint32_t> route; // list of node id that the neighbor will go
    std::vector<uint64_t> expectedArrival; // expected arrival time of each waypoint in route
    uint64_t lastContactTime;

    NeighborInfomation() : ip(0), lastContactTime(0) {}

};
#pragma endregion

#pragma region Class
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

    //* --- Một số hàm tiện ích ---
    // Lấy route của ferry, dưới dạng node IP
    virtual std::vector<uint32_t> GetServingNodeRoute() = 0;
    // Lấy route của ferry, dưới dạng waypoint
    virtual std::vector<point2D> GetServingWaypointRoute() = 0;

    std::vector<uint64_t> GetServingExpectedArrival();

    void SetVisitTime(uint32_t nodeIp, uint64_t time);
    void UpdateVisitTime(const std::unordered_map<uint32_t, uint64_t> receivedVisitTime);

    //* --- Các hàm gửi thông điệp (Sending functions) ---
    void Beacon();
    void SendGroundHello(Ipv4Address ferryIp);
    void SendFerryHello(Ipv4Address ferryIp);
    void SendBundle(Bundle& bundle, Ipv4Address neighborIp);
    void SendBundleAck(Bundle bundle, Ipv4Address neighborIp);
    void Schedule_FerryToGround_Transfer(Ipv4Address groundIp);
    void Schedule_GroundToFerry_Transfer(Ipv4Address ferryIp);

    //* --- Logic xử lý sự kiện dùng chung ---
    virtual void ReceivePacket(Ptr<Socket> socket);

    // virtual void OnNeighborDiscorver(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);

    virtual void OnGroundReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet);

    virtual void OnFerryReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveGroundHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet);


    //* --- Bundle Logic dùng chung ---
    void GenerateBundle();
    void RemoveExpiredBundles();
    void RemoveOldBundle();
    void RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp);
    void BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp);

    //* --- Variables (protected) ---
    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    uint8_t m_groupId; // for clustering
    uint8_t m_mode; // curent operation mode
    bool m_enableFerryComm; // enable communication between ferry 

    std::unordered_map<uint32_t, NeighborInfomation> m_neighbor;
    std::unordered_map<uint32_t, uint64_t> m_visitTime;

    double m_bundleGenRate;
    std::vector<Bundle> m_buffer;
    uint32_t m_maxBufferSize;
    uint32_t m_bundleIdCounter;

    Ptr<ConstantVelocityMobilityModel> m_mobility;
};

#pragma endregion

NS_OBJECT_ENSURE_REGISTERED(BaseDtnApp);

#pragma region Setup

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

#pragma endregion

#pragma region Helper
std::vector<uint64_t> BaseDtnApp::GetServingExpectedArrival() {
    auto waypointRoute = this->GetServingWaypointRoute();
    if (waypointRoute.empty()) return {};

    std::vector<uint64_t> expectedArrival;
    Vector3D currentPos = m_mobility->GetPosition();
    double currentTime = Simulator::Now().GetMicroSeconds();
    for (auto waypoint : waypointRoute) {
        point2D relative = { waypoint.x - currentPos.x,
                             waypoint.y - currentPos.y };
        double timeToReach = relative.length() * 1000000.0 / config.ferrySpeed;
        currentTime += timeToReach;
        expectedArrival.push_back(currentTime);
        currentPos = Vector3D(waypoint.x, waypoint.y, 0.0);
    }
    return expectedArrival;
}

void BaseDtnApp::UpdateVisitTime(const std::unordered_map<uint32_t, uint64_t> receivedVisitTime) {
    for (auto it = receivedVisitTime.begin(); it != receivedVisitTime.end(); ++it) {
        if (m_visitTime.find(it->first) == m_visitTime.end()) {
            m_visitTime[it->first] = it->second;
        }
        else {
            m_visitTime[it->first] = std::max(m_visitTime[it->first], it->second);
        }
    }
}

void BaseDtnApp::SetVisitTime(uint32_t nodeIp, uint64_t time) {
    m_visitTime[nodeIp] = time;
}

#pragma endregion
#pragma region SendingFunction

void BaseDtnApp::Beacon() {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_BEACON);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, broadcast);

    FerryVisualizer::logBeacon(nodeId[m_myIp.Get()]);

    Time nextBeacon = Seconds(m_rand->GetValue(0.0, config.beaconRandomnes)) + Seconds(config.beaconInterval);
    // Schedule next Hello
    Simulator::Schedule(nextBeacon, &BaseDtnApp::Beacon, this);
}

void BaseDtnApp::SendGroundHello(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::GROUND_HELLO);
    header.SetNodeIP(m_myIp);

    VisitTimeHeader vTimeHeader;
    vTimeHeader.FromMap(m_visitTime);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(vTimeHeader);
    packet->AddHeader(header);


    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendFerryHello(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_HELLO);
    header.SetNodeIP(m_myIp);

    VisitTimeHeader vTimeHeader;
    vTimeHeader.FromMap(m_visitTime);

    FerryRouteHeader fRouteHeader;
    fRouteHeader.SetGroup(m_groupId);
    auto routeIp = this->GetServingNodeRoute();
    auto routeArrival = this->GetServingExpectedArrival();
    uint32_t count = routeIp.size();
    fRouteHeader.SetCount(count);
    fRouteHeader.SetWaypoints(routeIp);
    fRouteHeader.SetExpectedArrival(routeArrival);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(fRouteHeader);
    packet->AddHeader(vTimeHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendBundle(Bundle& bundle, Ipv4Address neighborIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::BUNDLE);
    header.SetNodeIP(m_myIp);

    BundleHeader bundleHeader = BundleHeader::fromBundle(bundle);

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
    Simulator::Schedule(jitter, &BaseDtnApp::SendFerryHello, this, groundIp);
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

#pragma endregion

#pragma region OnReceive

void BaseDtnApp::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from))) {
        MessageTypeHeader topHeader;
        packet->RemoveHeader(topHeader);

        FerryVisualizer::logPacket(
            nodeId[topHeader.GetNodeIP().Get()], // from node id
            nodeId[m_myIp.Get()], // to node id
            topHeader.GetMetaName() // metadata
        );
        if (topHeader.GetNodeIP() == m_myIp) continue;

        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << nodeId[m_myIp.Get()] << " Receive " << topHeader.GetMetaName() << " from " << nodeId[topHeader.GetNodeIP().Get()]);

        auto packetType = topHeader.GetType();
        if (packetType == MessageTypeHeader::FERRY_BEACON) {
            OnGroundReceiveBeacon(topHeader.GetNodeIP(), packet);
        }
        else {
            uint64_t currentTime = Simulator::Now().GetMicroSeconds();
            m_neighbor[topHeader.GetNodeIP().Get()].lastContactTime = currentTime;
        }
        if (packetType == MessageTypeHeader::GROUND_HELLO) {
            OnFerryReceiveGroundHello(topHeader.GetNodeIP(), packet);
        }
        if (packetType == MessageTypeHeader::BUNDLE) {
            OnGroundReceiveBundle(topHeader.GetNodeIP(), packet);
            OnFerryReceiveBundle(topHeader.GetNodeIP(), packet);
        }
        if (packetType == MessageTypeHeader::BUNDLE_ACK) {
            OnGroundReceiveBundleAck(topHeader.GetNodeIP(), packet);
            OnFerryReceiveBundleAck(topHeader.GetNodeIP(), packet);
        }
        if (packetType == MessageTypeHeader::FERRY_HELLO) {
            OnGroundReceiveFerryHello(topHeader.GetNodeIP(), packet);
            OnFerryReceiveFerryHello(topHeader.GetNodeIP(), packet);
        }
    }
}

//* ----- GROUND -----

void BaseDtnApp::OnReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet) {
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();
    uint32_t source = sourceIp.Get();

    if (m_neighbor.find(source) != m_neighbor.end() &&  // not a new neighbor
        currentTime - m_neighbor[source].lastContactTime < config.contactTimeout) // still in contact
        return;

    m_neighbor[source].lastContactTime = currentTime;
    OnGroundReceiveBeacon(sourceIp, packet);
    OnFerryReceiveBeacon(sourceIp, packet);
}

void BaseDtnApp::OnGroundReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;

    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendGroundHello, this, sourceIp);
}

void BaseDtnApp::OnGroundReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;

    VisitTimeHeader vTimeHeader;
    packet->RemoveHeader(vTimeHeader);
    UpdateVisitTime(vTimeHeader.ToMap());

    FerryRouteHeader ferryRouteHeader;
    packet->RemoveHeader(ferryRouteHeader);

    Schedule_GroundToFerry_Transfer(sourceIp);
}

void BaseDtnApp::OnGroundReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;
    BundleHeader bundleHeader;
    packet->RemoveHeader(bundleHeader);
    Bundle bundle = bundleHeader.toBundle();

    if (bundle.destination != m_myIp) {
        NS_LOG_UNCOND("FATAL: all bundles send to ground node should be the final hop");
        NS_ASSERT(false);
        return;
    }

    NS_LOG_UNCOND("Bundle reached destination");
    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
    Report::bundleReachedDestination++;
    Report::totalHopReachedDestination += bundle.hop;
    Report::totalHop++;
    Report::bundleFowardCount++;
    Report::totalDelay += Simulator::Now() - MicroSeconds(bundle.creationTime);
}

void BaseDtnApp::OnGroundReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;
    BundleAckHeader bundleAckHeader;
    packet->RemoveHeader(bundleAckHeader);
    RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    if (nodeType[sourceIp.Get()] == NODE_TYPE_FERRY) {
        Schedule_GroundToFerry_Transfer(sourceIp);
    }
}

//* ----- FERRY -----

void BaseDtnApp::OnFerryReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;
    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendFerryHello, this, sourceIp);
}

void BaseDtnApp::OnFerryReceiveGroundHello(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;

    VisitTimeHeader vTimeHeader;
    packet->RemoveHeader(vTimeHeader);
    UpdateVisitTime(vTimeHeader.ToMap());

    Schedule_FerryToGround_Transfer(sourceIp);
}

void BaseDtnApp::OnFerryReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;

    VisitTimeHeader vTimeHeader;
    packet->RemoveHeader(vTimeHeader);
    UpdateVisitTime(vTimeHeader.ToMap());

    FerryRouteHeader ferryRouteHeader;
    packet->RemoveHeader(ferryRouteHeader);

    // TODO Ferry to Ferry transfer
}

void BaseDtnApp::OnFerryReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;
    BundleHeader bundleHeader;
    packet->RemoveHeader(bundleHeader);
    Bundle bundle = bundleHeader.toBundle();

    if (bundle.destination == m_myIp) {
        NS_LOG_UNCOND("FATAL: FERRY is not the destination of any bundle");
        NS_ASSERT(false);
        return;
    }

    RemoveExpiredBundles();
    if (m_buffer.size() >= m_maxBufferSize) {
        NS_LOG_UNCOND("Bundle dropped due to buffer overflow");
        return;
    }
    NS_LOG_UNCOND("Bundle added to buffer");

    Report::totalHop++;
    Report::bundleFowardCount++;
    m_buffer.push_back(bundle);
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    // RemoveOldBundle();

    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
}

void BaseDtnApp::OnFerryReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;
    BundleAckHeader bundleAckHeader;
    packet->RemoveHeader(bundleAckHeader);
    RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    if (nodeType[sourceIp.Get()] == NODE_TYPE_GROUND) {
        Schedule_FerryToGround_Transfer(sourceIp);
    }
    // TODO Ferry to Ferry transfer
}

#endif