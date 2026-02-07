#ifndef PIGEON_DTN_APP_H
#define PIGEON_DTN_APP_H

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

// ===========================================================================
// 2. APP & BUNDLE STORAGE (Có cập nhật Jitter)
// ===========================================================================

class PigeonDtnApp : public Application {
    public:
    PigeonDtnApp();
    virtual ~PigeonDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Node>node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType);
    void EnableBundleGeneration(double rate, bool inversed = true);
    void InitializeTSPMobility(const std::vector<point2D>& points, const std::vector<uint32_t>& order);
    void SetGroup(uint8_t groupId) {
        m_groupId = groupId;
    }

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
    void RemoveImpossibleBundles(); // remove bundles that event if a ferry fly straight from current position to the target, it still run out of TTL
    void RemoveOldBundle(); // remove oldest bundles to keep buffer size < max buffer size
    void RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp); // remove transmitted bundle

    void BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp);

    std::vector<std::vector<double>> GetDeadlines();

    // Mobility scheduling functions for ferry nodes
    EventId m_mobilityScheduleEvent;
    void ScheduleNextWaypoint();
    void ScheduleFerryWaypoint();
    void SchedulePigeonWaypoint();
    void SchedulePigeonOnBufferFull();

    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    uint8_t m_mode; // 0: Default/ground, 1: Ferry, 2: Pigeon
    uint8_t m_groupId; // for clustering

    double m_bundleGenRate;

    std::vector<Bundle> m_buffer;
    uint32_t m_maxBufferSize;
    uint32_t m_bundleIdCounter;

    std::vector<point2D> m_ferryPoints;
    std::vector<uint32_t> m_ferryOrder;
    std::uint32_t m_ferryNextIndex;

    // std::vector<point2D> m_pigeonPoints; // this is the same as ground node list
    std::vector<uint32_t> m_pigeonOrder;
    std::uint32_t m_pigeonNextIndex;

    Ptr<ConstantVelocityMobilityModel> m_mobility;

};

NS_OBJECT_ENSURE_REGISTERED(PigeonDtnApp);

TypeId PigeonDtnApp::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::PigeonDtnApp")
        .SetParent<Application>()
        .AddConstructor<PigeonDtnApp>();
    return tid;
}

PigeonDtnApp::PigeonDtnApp()
    : m_port(80), m_maxBufferSize(100), m_bundleIdCounter(0) {
}

PigeonDtnApp::~PigeonDtnApp() {}

void PigeonDtnApp::Setup(Ptr<Node> node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType) {
    m_node = node;
    m_socket = socket;
    m_myIp = myIp;
    m_maxBufferSize = bufferSize;
    m_nodeType = nodeType;
    m_mode = nodeType;
    m_bundleGenRate = 0.0;

    m_mobility = m_node->GetObject<ConstantVelocityMobilityModel>();

    m_buffer.clear();
    m_buffer.reserve(m_maxBufferSize + 1);
}

void PigeonDtnApp::InitializeTSPMobility(const std::vector<point2D>& points, const std::vector<uint32_t>& order) {
    // uint32_t startIndex = m_rand->GetInteger(0, order.size() - 1);
    auto cvmm = m_node->GetObject<ConstantVelocityMobilityModel>();
    m_ferryNextIndex = 0;
    Vector3D currentPos = cvmm->GetPosition();
    for (uint32_t i = 1; i < order.size(); i++) {
        point2D relative = { points[order[i]].x - currentPos.x,
                             points[order[i]].y - currentPos.y };
        point2D startIdRelative = { points[m_ferryNextIndex].x - currentPos.x,
                                    points[m_ferryNextIndex].y - currentPos.y };
        double dist1 = relative.x * relative.x + relative.y * relative.y;
        double dist2 = startIdRelative.x * startIdRelative.x + startIdRelative.y * startIdRelative.y;
        if (dist1 < dist2) {
            m_ferryNextIndex = i;
        }
    }
    m_ferryPoints = points;
    m_ferryOrder = order;
    m_mobilityScheduleEvent = Simulator::Schedule(Time(0), &PigeonDtnApp::ScheduleNextWaypoint, this);
}

/**
 * inversed:
 *  true -> rate: average seconds between 2 bundle : seconds / packet
 *  false -> packet geration rate : packets/second
 */
void PigeonDtnApp::EnableBundleGeneration(double rate, bool inversed) {
    if (inversed) {
        m_bundleGenRate = 1.0 / rate;
    }
    else {
        m_bundleGenRate = rate;
    }
    double averageBundleTime = 1.0 / m_bundleGenRate;
    double startTime = 0.1 + m_rand->GetValue(0.0, averageBundleTime);
    Simulator::Schedule(Seconds(startTime), &PigeonDtnApp::GenerateBundle, this);
}

void PigeonDtnApp::StartApplication(void)
{
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port)) == -1)
    {
        NS_LOG_UNCOND("Bind failed for node " << nodeId[m_myIp.Get()]);
        return;
    }
    m_socket->SetRecvCallback(MakeCallback(&PigeonDtnApp::ReceivePacket, this));

    // OCB hỗ trợ broadcast rất tốt qua IP broadcast
    m_socket->SetAllowBroadcast(true);

    NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " started listening on port " << m_port);

    if (m_nodeType == 0)
    {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a GROUND node.");
    }
    else
    {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << " is a FERRY node.");


        Simulator::Schedule(GetJitter(), &PigeonDtnApp::Beacon, this);
    }
}

void PigeonDtnApp::StopApplication(void) {
    if (m_socket) { m_socket->Close(); }
}

Time PigeonDtnApp::GetJitter() {
    return MicroSeconds(m_rand->GetInteger(config.jitterAmount, config.jitterAmount * 3));
}

void PigeonDtnApp::Beacon() {
    // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node " << nodeId[m_myIp.Get()] << ": Sending HELLO (OCB)");
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
    Simulator::Schedule(nextBeacon, &PigeonDtnApp::Beacon, this);
}

void PigeonDtnApp::SendGroundHello(Ipv4Address ferryIp) {
    MessageTypeHeader header;
    header.SetType(MessageTypeHeader::GROUND_HELLO);
    header.SetNodeIP(m_myIp);

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(ferryIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void PigeonDtnApp::SendFerryHello(Ipv4Address ferryIp) {
    // TODO
}

void PigeonDtnApp::SendFerryAcceptTransfer(Ipv4Address groundIp) {
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

void PigeonDtnApp::SendBundle(Bundle& bundle, Ipv4Address neighborIp) {
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
        &PigeonDtnApp::BundleAckTimeout, this, bundle.id, bundle.source.Get());

    Ptr<Packet> packet = Create<Packet>(0);
    packet->AddHeader(bundleHeader);
    packet->AddHeader(header);

    InetSocketAddress unicast = InetSocketAddress(neighborIp, m_port);
    m_socket->SendTo(packet, 0, unicast);
}

void PigeonDtnApp::SendBundleAck(Bundle bundle, Ipv4Address neighborIp) {
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

void PigeonDtnApp::ScheduleAcceptTransfer(Ipv4Address groundIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false && bundle.destination == groundIp) {
            Simulator::Schedule(jitter, &PigeonDtnApp::SendBundle, this, bundle, groundIp);
            return;
        }
    }
    if (m_buffer.size() >= m_maxBufferSize) {
        return; // there are no buffer left to receive additional bundle
    }
    Simulator::Schedule(jitter, &PigeonDtnApp::SendFerryAcceptTransfer, this, groundIp);
}

void PigeonDtnApp::ScheduleTransfer(Ipv4Address ferryIp) {
    Time jitter = GetJitter();
    RemoveExpiredBundles();

    if (m_groupId != nodeGroup[ferryIp.Get()]) {
        // send bundle that belongs to ground node with ferry group id
        // TODO send bundle knowing ferry route
        for (auto& bundle : m_buffer) {
            if (bundle.flag_waitingAck == false && nodeGroup[bundle.destination.Get()] == nodeGroup[ferryIp.Get()]) {
                Simulator::Schedule(jitter, &PigeonDtnApp::SendBundle, this, bundle, ferryIp);
                return;
            }
        }
        return; // not in the same group
    }
    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            Simulator::Schedule(jitter, &PigeonDtnApp::SendBundle, this, bundle, ferryIp);
            return;
        }
    }
}

void PigeonDtnApp::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        MessageTypeHeader topHeader;
        packet->RemoveHeader(topHeader);

        FerryVisualizer::logPacket(
            nodeId[topHeader.GetNodeIP().Get()], // from node id
            nodeId[m_myIp.Get()], // to node id
            topHeader.GetMetaName() // metadata
        );
        // Tự mình gửi thì bỏ qua (Loopback)
        if (topHeader.GetNodeIP() == m_myIp) continue;

        auto packetType = topHeader.GetType();
        if (packetType == MessageTypeHeader::FERRY_BEACON) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << nodeId[m_myIp.Get()]
                << ": BEACON from Ferry " << topHeader.GetNodeIP());
            Time jitter = GetJitter();
            if (m_nodeType == NODE_TYPE_GROUND) {
                Simulator::Schedule(jitter, &PigeonDtnApp::SendGroundHello, this, topHeader.GetNodeIP());
            }
            continue;
        }
        if (packetType == MessageTypeHeader::GROUND_HELLO) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << nodeId[m_myIp.Get()]
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
                Time jitter = GetJitter();
                Simulator::Schedule(jitter, &PigeonDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
                Report::bundleReachedDestination++;
                Report::totalHopReachedDestination += bundle.hop;
                Report::totalHop++;
                Report::bundleFowardCount++;
                Report::totalDelay += Simulator::Now() - MicroSeconds(bundle.creationTime);
                continue;
            }

            RemoveExpiredBundles();
            if (m_buffer.size() >= m_maxBufferSize) {
                NS_LOG_UNCOND("Bundle dropped due to buffer overflow");
                if (m_nodeType == NODE_TYPE_FERRY && m_mode == MODE_FERRY) {
                    SchedulePigeonOnBufferFull();
                }
                continue; // there are no place for this bundle
            }
            NS_LOG_UNCOND("Bundle added to buffer");

            Report::totalHop++;
            Report::bundleFowardCount++;
            m_buffer.push_back(bundle);
            FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
            // RemoveOldBundle();
            if (m_nodeType == NODE_TYPE_FERRY && m_mode == MODE_FERRY && m_buffer.size() >= m_maxBufferSize) {
                SchedulePigeonOnBufferFull();
            }

            Time jitter = GetJitter();
            Simulator::Schedule(jitter, &PigeonDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
            continue;
        }
        if (packetType == MessageTypeHeader::BUNDLE_ACK) {
            BundleAckHeader bundleAckHeader;
            packet->RemoveHeader(bundleAckHeader);
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << nodeId[m_myIp.Get()]
                << ": BUNDLE ACK from " << topHeader.GetNodeIP());
            RemoveAckedBundle(bundleAckHeader.GetBundleId(), bundleAckHeader.GetSourceIp().Get());
            FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
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
                << "s Node " << nodeId[m_myIp.Get()]
                << ": ACCEPT TRANSFER from ferry node " << topHeader.GetNodeIP()
            );
            ScheduleTransfer(topHeader.GetNodeIP());
        }
    }

}

void PigeonDtnApp::GenerateBundle() {
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
    Simulator::Schedule(Seconds(IAT), &PigeonDtnApp::GenerateBundle, this);
}

void PigeonDtnApp::RemoveExpiredBundles() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();
    while (m_buffer.size() > 0 && m_buffer[0].creationTime + config.bundleTTL < currentTime) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": EXPIRED Bundle " << m_buffer[0].id);
        m_buffer.erase(m_buffer.begin());
    }
    RemoveImpossibleBundles();
}

void PigeonDtnApp::RemoveImpossibleBundles() {
    Vector3D currentPos = m_mobility->GetPosition();
    bool removed_bundle = true;
    while (removed_bundle) {
        removed_bundle = false;
        for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
            point2D target = { groundNodePos[it->destination.Get()].x, groundNodePos[it->destination.Get()].y };
            point2D relative = { target.x - currentPos.x, target.y - currentPos.y };
            double distance = relative.length() - config.commRange;
            if (m_nodeType == NODE_TYPE_GROUND)
                distance -= config.commRange;
            Time expectedArrival = Simulator::Now() + Seconds(distance / config.ferrySpeed);
            Time bundleExpirationTime = MicroSeconds(it->creationTime + config.bundleTTL);
            if (expectedArrival > bundleExpirationTime) {
                NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": EXPIRED Bundle " << it->id);
                m_buffer.erase(it);
                removed_bundle = true;
                break;
            }
        }
    }
}

void PigeonDtnApp::RemoveOldBundle() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    while (m_buffer.size() > m_maxBufferSize) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": DROPPED Bundle " << m_buffer[0].id);
        m_buffer.erase(m_buffer.begin());
    }
}

void PigeonDtnApp::RemoveAckedBundle(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            m_buffer.erase(it);
            return;
        }
    }
}

void PigeonDtnApp::BundleAckTimeout(uint32_t bundleId, uint32_t sourceIp) {
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->id == bundleId && it->source.Get() == sourceIp) {
            it->flag_waitingAck = false; // set flag = false so bundle can be transfered again later
            return;
        }
    }
}


std::vector<std::vector<double>> PigeonDtnApp::GetDeadlines() {
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


void PigeonDtnApp::ScheduleNextWaypoint() {
    if (m_mode == MODE_FERRY) {
        ScheduleFerryWaypoint();
    }
    else if (m_mode == MODE_PIGEON) {
        SchedulePigeonWaypoint();
    }
    else {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
    }

}

void PigeonDtnApp::ScheduleFerryWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();

    point2D relative = { m_ferryPoints[m_ferryOrder[m_ferryNextIndex]].x - currentPos.x,
                         m_ferryPoints[m_ferryOrder[m_ferryNextIndex]].y - currentPos.y };

    double distance = relative.length();
    double time = distance / config.ferrySpeed;

    double currentTime = Simulator::Now().GetSeconds();
    auto deadlines = GetDeadlines();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    std::vector<uint32_t> pigeonRoute;
    pigeonRoute = TSPDeadlineBasedGA(
        groundNodePos,
        deadlines,
        m_ferryPoints[m_ferryOrder[m_ferryNextIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed
    );
    pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
        groundNodePos,
        deadlines,
        m_ferryPoints[m_ferryOrder[m_ferryNextIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed,
        pigeonRoute
    );

    uint32_t cost = ComputeDeadlineCost(
        pigeonRoute,
        groundNodePos,
        deadlines,
        m_ferryPoints[m_ferryOrder[m_ferryNextIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed
    );

    if (cost < m_buffer.size()) { // cannot sastify all deadlines -> switch to pigeon mode
        pigeonRoute = TSPDeadlineBasedGA(
            groundNodePos,
            deadlines,
            { currentPos.x, currentPos.y }, // starting pos
            currentTime,
            config.ferrySpeed
        );
        pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
            groundNodePos,
            deadlines,
            { currentPos.x, currentPos.y }, // starting pos
            currentTime,
            config.ferrySpeed,
            pigeonRoute
        );
        m_mode = MODE_PIGEON;
        m_pigeonOrder = pigeonRoute;
        m_pigeonNextIndex = 0;
        SchedulePigeonWaypoint();
        NS_LOG_UNCOND("Switch to pigeon mode");
        return;
    }
    else {
        FerryVisualizer::logRoute(nodeId[m_myIp.Get()], FerryVisualizer::tspRouteHelper(m_ferryPoints, m_ferryOrder, m_ferryNextIndex));

        m_ferryNextIndex = (m_ferryNextIndex + 1) % m_ferryOrder.size();

        if (time < 0.1) {
            // handle special case: when there only one node
            // ferry fly from its starting position to the only node then stay there, check every 1 seconds 
            m_mobilityScheduleEvent = Simulator::Schedule(Seconds(1.0), &PigeonDtnApp::ScheduleNextWaypoint, this);
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            return;
        }


        m_mobilityScheduleEvent = Simulator::Schedule(Seconds(time), &PigeonDtnApp::ScheduleNextWaypoint, this);

        point2D velocity;
        velocity.x = relative.x / distance * config.ferrySpeed;
        velocity.y = relative.y / distance * config.ferrySpeed;

        m_mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));
    }
}

void PigeonDtnApp::SchedulePigeonWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();
    if (m_pigeonNextIndex >= m_pigeonOrder.size()) {
        m_mode = MODE_FERRY;
        ScheduleFerryWaypoint();
        NS_LOG_UNCOND("Switch to ferry mode");
        return;
    }
    point2D relative = { groundNodePos[m_pigeonOrder[m_pigeonNextIndex]].x - currentPos.x,
                         groundNodePos[m_pigeonOrder[m_pigeonNextIndex]].y - currentPos.y };

    double distance = relative.length();
    double time = distance / config.ferrySpeed;
    FerryVisualizer::logRoute(
        nodeId[m_myIp.Get()],
        FerryVisualizer::tspRouteHelper(groundNodePos, m_pigeonOrder, m_pigeonNextIndex, false));

    m_pigeonNextIndex++;

    if (time < 0.1) {
        m_mobilityScheduleEvent = Simulator::Schedule(Seconds(1.0), &PigeonDtnApp::ScheduleNextWaypoint, this);
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        return;
    }
    m_mobilityScheduleEvent = Simulator::Schedule(Seconds(time), &PigeonDtnApp::ScheduleNextWaypoint, this);

    point2D velocity;
    velocity.x = relative.x / distance * config.ferrySpeed;
    velocity.y = relative.y / distance * config.ferrySpeed;

    m_mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));
}

void PigeonDtnApp::SchedulePigeonOnBufferFull() {
    Vector3D currentPos = m_mobility->GetPosition();
    double currentTime = Simulator::Now().GetSeconds();
    auto deadlines = GetDeadlines();

    std::vector<uint32_t> pigeonRoute;
    pigeonRoute = TSPDeadlineBasedGA(
        groundNodePos,
        deadlines,
        { currentPos.x, currentPos.y }, // starting pos
        currentTime,
        config.ferrySpeed
    );
    pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
        groundNodePos,
        deadlines,
        { currentPos.x, currentPos.y }, // starting pos
        currentTime,
        config.ferrySpeed,
        pigeonRoute
    );
    m_mode = MODE_PIGEON;
    m_pigeonOrder = pigeonRoute;
    m_pigeonNextIndex = 0;
    SchedulePigeonWaypoint();
    NS_LOG_UNCOND("Buffer full, Switch to pigeon mode");
    return;
}

#endif // PIGEON_DTN_APP_H
