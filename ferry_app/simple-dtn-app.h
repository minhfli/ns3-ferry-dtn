#ifndef SIMPLE_DTN_APP_H
#define SIMPLE_DTN_APP_H

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

class SimpleDtnApp : public Application
{
    public:
    SimpleDtnApp();
    virtual ~SimpleDtnApp();

    static TypeId GetTypeId(void);
    void Setup(Ptr<Node>node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType);
    void EnableBundleGeneration(double rate, bool inversed = true);
    void InitializeTSPMobility(const std::vector<point2D>& points, const std::vector<uint32_t>& order);

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

    // Mobility scheduling functions for ferry nodes
    void ScheduleNextWaypoint(const std::vector<point2D>& points,
        const std::vector<uint32_t>& order,
        double speed,
        uint32_t index,
        Ptr<ConstantVelocityMobilityModel> mobility);

    Ptr<Node> m_node;

    Ptr<Socket> m_socket;
    Ipv4Address m_myIp;
    uint16_t m_port;
    uint8_t m_nodeType; // 0: Ground, 1: Ferry
    // EventId m_helloEvent;

    double m_bundleGenRate;

    std::vector<Bundle> m_buffer;
    uint32_t m_maxBufferSize;
    uint32_t m_bundleIdCounter;


    Ptr<MobilityModel> m_mobility;

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

void SimpleDtnApp::Setup(Ptr<Node> node, Ptr<Socket> socket, Ipv4Address myIp, uint32_t bufferSize, uint8_t nodeType)
{
    m_node = node;
    m_socket = socket;
    m_myIp = myIp;
    m_maxBufferSize = bufferSize;
    m_nodeType = nodeType;
    m_bundleGenRate = 0.0;

    m_buffer.clear();

    if (m_nodeType == NODE_TYPE_FERRY)
        m_mobility = m_node->GetObject<ConstantVelocityMobilityModel>();
    else {
        m_mobility = m_node->GetObject<ConstantPositionMobilityModel>();
    }
}

void SimpleDtnApp::InitializeTSPMobility(const std::vector<point2D>& points, const std::vector<uint32_t>& order) {
    uint32_t startIndex = m_rand->GetInteger(0, order.size() - 1);
    auto cvmm = m_node->GetObject<ConstantVelocityMobilityModel>();
    Simulator::Schedule(Time(0), &SimpleDtnApp::ScheduleNextWaypoint, this, points, order, config.ferrySpeed, startIndex, cvmm);
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
        NS_LOG_UNCOND("Bind failed for node " << nodeId[m_myIp.Get()]);
        return;
    }
    m_socket->SetRecvCallback(MakeCallback(&SimpleDtnApp::ReceivePacket, this));

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
    RemoveExpiredBundles();

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
    RemoveExpiredBundles();

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
                Simulator::Schedule(jitter, &SimpleDtnApp::SendGroundHello, this, topHeader.GetNodeIP());
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
                Simulator::Schedule(jitter, &SimpleDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
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
                continue; // there are no place for this bundle
            }
            NS_LOG_UNCOND("Bundle added to buffer");

            Report::totalHop++;
            Report::bundleFowardCount++;
            m_buffer.push_back(bundle);
            FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
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
    Report::bundleCount++;

    RemoveOldBundle();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": CREATED Bundle " << b.id << " to " << b.destination);

    // generate based on poisson distribution
    double jitter = m_rand->GetValue(0.0, 1.0);
    double IAT = -log(1.0 - jitter) / m_bundleGenRate;
    Simulator::Schedule(Seconds(IAT), &SimpleDtnApp::GenerateBundle, this);
}

void SimpleDtnApp::RemoveExpiredBundles() {
    uint64_t currentTime = Simulator::Now().GetMicroSeconds();

    m_buffer.erase(std::remove_if(m_buffer.begin(), m_buffer.end(), [&](const Bundle& b) {
        // Điều kiện 1: Hết TTL
        if (b.creationTime + config.bundleTTL < currentTime) return true;

        // Điều kiện 2: Impossible (nếu bay thẳng vẫn không kịp)
        Vector3D currentPos = m_mobility->GetPosition();
        point2D target = groundNodePos[rawNodeId(b.destination.Get())];
        double dist = point2D{ target.x - currentPos.x, target.y - currentPos.y }.length() - config.commRange;
        if (m_nodeType == NODE_TYPE_GROUND) dist -= config.commRange;

        double timeToReach = dist / config.ferrySpeed;
        return (Simulator::Now() + Seconds(timeToReach) > MicroSeconds(b.creationTime + config.bundleTTL));
        }), m_buffer.end());

}

void SimpleDtnApp::RemoveOldBundle() {
    std::sort(m_buffer.begin(), m_buffer.end(), compareBundleTime);
    while (m_buffer.size() > m_maxBufferSize) {
        NS_LOG_UNCOND("Node " << nodeId[m_myIp.Get()] << ": DROPPED Bundle " << m_buffer[0].id);
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

void SimpleDtnApp::ScheduleNextWaypoint(const std::vector<point2D>& points,
    const std::vector<uint32_t>& order,
    double speed,
    uint32_t index, // curent index
    Ptr<ConstantVelocityMobilityModel> mobility
) {

    uint32_t nextIndex = (index + 1) % order.size();

    Vector3D currentPos = mobility->GetPosition();


    point2D relative = { points[order[nextIndex]].x - currentPos.x,
                         points[order[nextIndex]].y - currentPos.y };

    double distance = std::sqrt(relative.x * relative.x + relative.y * relative.y);
    double time = distance / speed;

    Simulator::Schedule(Seconds(time), &SimpleDtnApp::ScheduleNextWaypoint, this, points, order, speed, nextIndex, mobility);

    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], FerryVisualizer::tspRouteHelper(points, order, nextIndex));

    point2D velocity;
    velocity.x = relative.x / distance * speed;
    velocity.y = relative.y / distance * speed;

    mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));

}



#endif
