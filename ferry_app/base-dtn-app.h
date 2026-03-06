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
#include <map>
#include <unordered_set>
#include <set>

using namespace ns3;

Time GetJitter() {
    return MicroSeconds(m_rand->GetInteger(config.jitterAmount, config.jitterAmount * 3));
}
#pragma region Structs

struct NeighborInfomation {
    std::vector<uint32_t> route; // list of node id that the neighbor will go
    std::vector<uint64_t> expectedArrival; // expected arrival time of each waypoint in route
    uint64_t lastContactTime;
    std::map<uint32_t, uint32_t> bufferState; // buffer state which is a map of ip and count of bundle that need to reach it
    uint8_t operationMode;
    uint8_t group;

    NeighborInfomation() : lastContactTime(0) {}

};
#pragma endregion

#pragma region Class
class BaseDtnApp : public Application {
    private:
    constexpr static uint8_t FTF_MODE_RECEIVED_ACCEPT = 0;
    constexpr static uint8_t FTF_MODE_NOT_RECEIVED_ACCEPT = 1;

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
    void UpdateGroup(uint8_t groupId);
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
    // Lấy route của ferry, dưới dạng waypoint, mặc định trả về cá vị trí của node trong node route
    virtual std::vector<point2D> GetServingWaypointRoute();

    std::vector<uint64_t> GetServingExpectedArrival();
    std::map<uint32_t, uint32_t> GetBundleCount() const;
    std::vector<std::vector<double>> GetDeadlines();
    // Lấy danh sách node mà neighbor ferry sẽ đến thăm trước ferry hiện tại
    std::set<uint32_t> GetFasterNeighborWaypoints(NeighborInfomation neighbor);


    void SetVisitTime(uint32_t nodeIp, uint64_t time);
    void UpdateVisitTime(const std::unordered_map<uint32_t, uint64_t> receivedVisitTime);

    //* --- Các hàm gửi thông điệp (Sending functions) ---
    void Beacon();
    void SendGroundHello(Ipv4Address ferryIp);
    void SendFerryHello(Ipv4Address ferryIp);
    void SendFerryAcceptTransfer(Ipv4Address ferryIp);
    void SendBundle(Bundle& bundle, Ipv4Address neighborIp);
    void SendBundleAck(Bundle bundle, Ipv4Address neighborIp);
    void SendBundleAckAndAcceptTransfer(Bundle bundle, Ipv4Address neighborIp);

    void Schedule_FerryToGround_Transfer(Ipv4Address groundIp);
    void Schedule_GroundToFerry_Transfer(Ipv4Address ferryIp);
    // void Schedule_FerryToFerry_Transfer(Ipv4Address ferryIp, uint8_t mode);


    //* --- Logic xử lý sự kiện dùng chung ---
    virtual void ReceivePacket(Ptr<Socket> socket);

    // virtual void OnNeighborDiscorver(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);

    virtual void OnGroundReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveFerryAcceptTransfer(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnGroundReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet);

    virtual void OnFerryReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveGroundHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveFerryAcceptTransfer(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveBundle(Ipv4Address sourceIp, Ptr<Packet> packet);
    virtual void OnFerryReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet);

    //* --- Bundle Logic dùng chung ---
    void GenerateBundle();
    void BundleReachedDestination(Bundle b);
    void RemoveExpiredBundles();
    void RemoveOldBundle();
    Bundle RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp);
    void BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp);

    virtual Bundle* GroundSelectBundleToFerry(Ipv4Address neighborIp);
    virtual Bundle* FerrySelectBundleToGround(Ipv4Address neighborIp);
    virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp);

    virtual void BundleAckMobilityCallBack(Bundle b) {}

    //* --- Variables (protected) ---
    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    uint8_t m_groupId; // for clustering
    uint8_t m_mode; // curent operation mode

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
    nodeGenRate[m_myIp.Get()] = m_bundleGenRate;
    double averageBundleTime = 1.0 / m_bundleGenRate;
    double startTime = 0.1 + bundleGenRand->GetValue(0.0, averageBundleTime);
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

    if (m_nodeType == NODE_TYPE_GROUND) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a GROUND node.");
    }
    else {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a FERRY node.");
        Simulator::Schedule(Seconds(m_rand->GetValue(0.1, config.beaconInterval)), &BaseDtnApp::Beacon, this);
    }
}

void BaseDtnApp::StopApplication(void) {
    if (m_socket) { m_socket->Close(); }
}

void BaseDtnApp::UpdateGroup(uint8_t groupId) {
    m_groupId = groupId;
    nodeGroup[m_myIp.Get()] = groupId;
}

#pragma endregion

#pragma region Helper

std::vector<point2D> BaseDtnApp::GetServingWaypointRoute() {
    auto nodeRoute = this->GetServingNodeRoute();
    if (nodeRoute.empty()) return {};

    std::vector<point2D> route;
    for (auto node : nodeRoute) {
        route.push_back(nodePos(node));
    }
    return route;
}

std::vector<uint64_t> BaseDtnApp::GetServingExpectedArrival() {
    auto waypointRoute = this->GetServingWaypointRoute();
    if (waypointRoute.empty()) return {};

    std::vector<uint64_t> expectedArrival;
    Vector3D currentPos = m_mobility->GetPosition();
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();
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

std::map<uint32_t, uint32_t> BaseDtnApp::GetBundleCount() const {
    std::map<uint32_t, uint32_t> bundleCount;
    for (auto ip : groundNodeIps) {
        bundleCount[ip.Get()] = 0;
    }
    for (auto bundle : m_buffer) {
        bundleCount[bundle.destination.Get()]++;
    }
    return bundleCount;
}

std::vector<std::vector<double>> BaseDtnApp::GetDeadlines() {
    RemoveExpiredBundles();
    std::vector<std::vector<double>> deadlines;
    deadlines.resize(config.nGrounds);
    for (auto bundle : m_buffer) {
        uint32_t node = rawNodeId(bundle.destination.Get());
        double dl = bundle.creationTime + config.bundleTTL; //microsec
        dl /= 1000000.0;
        deadlines[node].push_back(dl);
    }
    return deadlines;
}

std::set<uint32_t> BaseDtnApp::GetFasterNeighborWaypoints(NeighborInfomation neighbor) {
    std::set<uint32_t> fasterWaypoints;

    for (uint32_t i = 0; i < neighbor.route.size(); i++) {
        uint32_t node = neighbor.route[i];
        double expect2 = (double)neighbor.expectedArrival[i] / 1000000.0;
        double expect1 = (double)CalExpectedArrival(node, GetServingNodeRoute(), GetServingExpectedArrival()) / 1000000.0;
        if (expect1 == 0 || expect1 - expect2 > config.minExpectedArrivalDifference)
            // neighbor go to node that not in current route or go faster
            fasterWaypoints.insert(node);
    }
    return fasterWaypoints;
}
#pragma endregion
#pragma region Sending 
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
    fRouteHeader.SetMode(m_mode);
    auto routeIp = this->GetServingNodeRoute();
    auto routeArrival = this->GetServingExpectedArrival();
    uint32_t count = routeIp.size();
    fRouteHeader.SetCount(count);
    fRouteHeader.SetWaypoints(routeIp);
    fRouteHeader.SetExpectedArrival(routeArrival);

    BufferStateHeader bufferStateHeader;
    bufferStateHeader.SetCapacity(m_maxBufferSize);
    bufferStateHeader.FromBuffer(m_buffer);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bufferStateHeader);
    packet->AddHeader(fRouteHeader);
    packet->AddHeader(vTimeHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendFerryAcceptTransfer(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::FERRY_ACCEPT_TRANSFER);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
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
    RemoveExpiredBundles();

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleAckHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void BaseDtnApp::SendBundleAckAndAcceptTransfer(Bundle bundle, Ipv4Address neighborIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::BUNDLE_ACK_FAT);
    header.SetNodeIP(m_myIp);

    BundleAckHeader bundleAckHeader;
    bundleAckHeader.SetBundleId(bundle.id);
    bundleAckHeader.SetSourceIp(bundle.source);
    bundleAckHeader.SetDestIp(neighborIp);
    RemoveExpiredBundles();

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleAckHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

#pragma endregion

#pragma region Scheduling

Bundle* BaseDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    if (m_groupId != nodeGroup[neighborIp.Get()]) {
        // send bundle that belongs to ground node with ferry group id
        for (auto& bundle : m_buffer) {
            if (bundle.flag_waitingAck == false && nodeGroup[bundle.destination.Get()] == nodeGroup[neighborIp.Get()]) {
                return &bundle;
            }
        }
        return nullptr; // not in the same group
    }
    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            return &bundle;
        }
    }
    return nullptr; // no bundle to send
}

Bundle* BaseDtnApp::FerrySelectBundleToGround(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false && bundle.destination == neighborIp) {
            return &bundle;
        }
    }
    return nullptr;
}

Bundle* BaseDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    // filter node that neighbor will go to it faster
    std::vector<uint32_t> nodeFilter; // first: nodeip, second: count
    auto neighbor = m_neighbor[neighborIp.Get()];

    for (uint32_t i = 0; i < neighbor.route.size(); i++) {
        uint32_t node = neighbor.route[i];
        double expect2 = (double)neighbor.expectedArrival[i] / 1000000.0;
        double expect1 = (double)CalExpectedArrival(node, GetServingNodeRoute(), GetServingExpectedArrival()) / 1000000.0;
        if (expect1 == 0 || expect1 - expect2 > config.minExpectedArrivalDifference)
            // neighbor go to node that not in current route or go faster
            nodeFilter.push_back(node);
    }

    for (Bundle& bundle : m_buffer) {
        if (bundle.flag_waitingAck) continue;
        for (uint32_t node : nodeFilter) {
            if (bundle.destination.Get() == node) {
                return &bundle;
            }
        }
    }

    return nullptr;
}

void BaseDtnApp::Schedule_FerryToGround_Transfer(Ipv4Address groundIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    auto bundlePtr = FerrySelectBundleToGround(groundIp);
    if (bundlePtr != nullptr) {
        Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, *bundlePtr, groundIp);
        return;
    }

    Simulator::Schedule(jitter, &BaseDtnApp::SendFerryHello, this, groundIp);

}

void BaseDtnApp::Schedule_GroundToFerry_Transfer(Ipv4Address ferryIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    auto bundlePtr = GroundSelectBundleToFerry(ferryIp);
    if (bundlePtr == nullptr)
        return; // no bundle to send
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, *bundlePtr, ferryIp);
}

#pragma endregion

#pragma region Bundle Logic

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
    Report::nodeBundleCount[rawNodeId(m_myIp.Get())][rawNodeId(dest.Get())]++;

    RemoveOldBundle();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": CREATED Bundle " << b.id << " to " << nodeId[dest.Get()]);

    // generate based on poisson distribution
    double jitter = bundleGenRand->GetValue(0.0, 1.0);
    double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    Simulator::Schedule(Seconds(IAT), &BaseDtnApp::GenerateBundle, this);
}

void BaseDtnApp::BundleReachedDestination(Bundle b) {
    Report::bundleReachedDestination++;
    Report::nodeBundleReachedDestination[rawNodeId(b.source.Get())][rawNodeId(b.destination.Get())]++;
    Report::totalHopReachedDestination += b.hop;
    Report::totalHop++;
    Report::bundleFowardCount++;
    Report::totalDelay += Simulator::Now() - MicroSeconds(b.creationTime);
    Report::delayList.push_back(Simulator::Now() - MicroSeconds(b.creationTime));
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

Bundle BaseDtnApp::RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp) {
    Bundle b;
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            b = *it;
            m_buffer.erase(it);
            return b;
        }
    }
    NS_LOG_UNCOND("Bundle to ack not found in buffer");

    // NS_ASSERT_MSG(false, "Bundle to ack not found in buffer");
    return b;
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
            OnReceiveBeacon(topHeader.GetNodeIP(), packet);
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
        if (packetType == MessageTypeHeader::BUNDLE_ACK_FAT) {
            OnGroundReceiveBundleAck(topHeader.GetNodeIP(), packet);
            OnFerryReceiveBundleAck(topHeader.GetNodeIP(), packet);
            OnGroundReceiveFerryAcceptTransfer(topHeader.GetNodeIP(), packet);
            OnFerryReceiveFerryAcceptTransfer(topHeader.GetNodeIP(), packet);
        }
        if (packetType == MessageTypeHeader::FERRY_ACCEPT_TRANSFER) {
            // OnGroundReceiveFerryAcceptTransfer(topHeader.GetNodeIP(), packet); // ground node wont get this message
            OnFerryReceiveFerryAcceptTransfer(topHeader.GetNodeIP(), packet);
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
    m_neighbor[sourceIp.Get()].route = ferryRouteHeader.GetWaypoints();
    m_neighbor[sourceIp.Get()].expectedArrival = ferryRouteHeader.GetExpectedArrival();
    m_neighbor[sourceIp.Get()].group = ferryRouteHeader.GetGroup();
    m_neighbor[sourceIp.Get()].operationMode = ferryRouteHeader.GetMode();


    BufferStateHeader bufferStateHeader;
    packet->RemoveHeader(bufferStateHeader);
    m_neighbor[sourceIp.Get()].bufferState = bufferStateHeader.ToCountMap();

    if (!bufferStateHeader.IsFull()) {
        Schedule_GroundToFerry_Transfer(sourceIp);
    }
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

    // NS_LOG_UNCOND("Bundle reached destination");
    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
    BundleReachedDestination(bundle);

}

void BaseDtnApp::OnGroundReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;
    BundleAckHeader bundleAckHeader;
    packet->RemoveHeader(bundleAckHeader);
    RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
}

void BaseDtnApp::OnGroundReceiveFerryAcceptTransfer(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_FERRY) return;
    if (nodeType[sourceIp.Get()] == NODE_TYPE_FERRY) {
        Schedule_GroundToFerry_Transfer(sourceIp);
    }
}

//* ----- FERRY -----

void BaseDtnApp::OnFerryReceiveBeacon(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (!config.enableFerryComm) return;
    if (m_nodeType == NODE_TYPE_GROUND) return;

    if (m_myIp < sourceIp) { // smaller ip node start the protocol
        Time jitter = GetJitter();
        Simulator::Schedule(jitter, &BaseDtnApp::SendFerryHello, this, sourceIp);
    }
}

void BaseDtnApp::OnFerryReceiveGroundHello(Ipv4Address sourceIp, Ptr<Packet> packet) {

    if (m_nodeType == NODE_TYPE_GROUND) return;

    VisitTimeHeader vTimeHeader;
    packet->RemoveHeader(vTimeHeader);
    UpdateVisitTime(vTimeHeader.ToMap());

    Schedule_FerryToGround_Transfer(sourceIp);
}

void BaseDtnApp::OnFerryReceiveFerryHello(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (!config.enableFerryComm) return;
    if (m_nodeType == NODE_TYPE_GROUND) return;

    VisitTimeHeader vTimeHeader;
    packet->RemoveHeader(vTimeHeader);
    UpdateVisitTime(vTimeHeader.ToMap());

    FerryRouteHeader ferryRouteHeader;
    packet->RemoveHeader(ferryRouteHeader);
    m_neighbor[sourceIp.Get()].route = ferryRouteHeader.GetWaypoints();
    m_neighbor[sourceIp.Get()].expectedArrival = ferryRouteHeader.GetExpectedArrival();
    m_neighbor[sourceIp.Get()].group = ferryRouteHeader.GetGroup();
    m_neighbor[sourceIp.Get()].operationMode = ferryRouteHeader.GetMode();

    BufferStateHeader bufferStateHeader;
    packet->RemoveHeader(bufferStateHeader);
    m_neighbor[sourceIp.Get()].bufferState = bufferStateHeader.ToCountMap();

    Time jitter = GetJitter();

    if (sourceIp < m_myIp) { // higher ip node return the hello
        Simulator::Schedule(jitter, &BaseDtnApp::SendFerryHello, this, sourceIp);
        return;
    }
    if (bufferStateHeader.IsFull()) { // neighbor cannot accept transfer
        if (m_buffer.size() < m_maxBufferSize) { // can accept transfer, send hello to trigger transfer
            Simulator::Schedule(jitter, &BaseDtnApp::SendFerryAcceptTransfer, this, sourceIp);
            return;
        }
        return;
    }

    Bundle* bundlePtr = FerrySelectBundleToFerry(sourceIp);
    if (bundlePtr == nullptr) {
        Simulator::Schedule(jitter, &BaseDtnApp::SendFerryAcceptTransfer, this, sourceIp);
        return;
    }
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, *bundlePtr, sourceIp);
}

void BaseDtnApp::OnFerryReceiveFerryAcceptTransfer(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;

    Bundle* bundlePtr = FerrySelectBundleToFerry(sourceIp);
    if (bundlePtr == nullptr) { // have no bundle to send, neighbor also have no bundle to send, end the protcol
        return;
    }
    Time jitter = GetJitter();
    Simulator::Schedule(jitter, &BaseDtnApp::SendBundle, this, *bundlePtr, sourceIp);
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

    bool hasBuffer = (m_buffer.size() < m_maxBufferSize);

    Time jitter = GetJitter();

    if (nodeType[sourceIp.Get()] == NODE_TYPE_GROUND) {
        if (hasBuffer) {
            Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAckAndAcceptTransfer, this, bundle, sourceIp);
        }
        else {
            Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
        }
        return;
    }
    if (nodeType[sourceIp.Get()] == NODE_TYPE_FERRY) {
        Time jitter2 = GetJitter() + jitter;
        Bundle* bundlePtr = FerrySelectBundleToFerry(sourceIp);
        if (bundlePtr == nullptr) {
            if (hasBuffer) {
                Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAckAndAcceptTransfer, this, bundle, sourceIp);
            }
            else {
                Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
            }
        }
        else {
            Simulator::Schedule(jitter, &BaseDtnApp::SendBundleAck, this, bundle, sourceIp);
            Simulator::Schedule(jitter2, &BaseDtnApp::SendBundle, this, *bundlePtr, sourceIp);
        }
        return;
    }

}

void BaseDtnApp::OnFerryReceiveBundleAck(Ipv4Address sourceIp, Ptr<Packet> packet) {
    if (m_nodeType == NODE_TYPE_GROUND) return;
    BundleAckHeader bundleAckHeader;
    packet->RemoveHeader(bundleAckHeader);

    Bundle bundleCopy = RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
    RemoveExpiredBundles();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    BundleAckMobilityCallBack(bundleCopy);

    if (nodeType[sourceIp.Get()] == NODE_TYPE_GROUND) {
        Schedule_FerryToGround_Transfer(sourceIp);
    }
}

#endif