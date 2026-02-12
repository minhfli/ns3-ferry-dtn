#ifndef SIMPLE_DTN_APP_H
#define SIMPLE_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace SIRA {
    bool created = false;
    std::vector<uint32_t> route;
    std::vector<point2D> points;
    std::vector<uint32_t> nodeList;
    void createRoute() {
        if (created) return;
        created = true;
        route = TSPClassicGA(points);
        route = TSPTwoOptOptimize(points, route);
    }
};

class SingleRouteDtnApp : public BaseDtnApp {
    public:
    SingleRouteDtnApp() : BaseDtnApp() {};
    virtual ~SingleRouteDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SingleRouteDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<SingleRouteDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual std::vector<point2D> GetServingWaypointRoute() override;

    private:

    // --- CORE LOGIC: Nơi bạn sẽ cài thuật toán Routing --)

    virtual void ReceivePacket(Ptr<Socket> socket);

    uint32_t m_nextWaypointIndex;
    void ScheduleNextWaypoint();
};

NS_OBJECT_ENSURE_REGISTERED(SingleRouteDtnApp);

void SingleRouteDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    if (!SIRA::created) {
        std::vector<point2D> points;
        for (auto index : servingNodesIndex) {
            points.push_back(groundNodePos[index]);
            SIRA::nodeList.push_back(groundNodeIps[index].Get());
        }
        SIRA::points = points;
        SIRA::createRoute();
    }
    m_nextWaypointIndex = m_rand->GetInteger(0, SIRA::route.size() - 1);
    Simulator::Schedule(Seconds(0), &SingleRouteDtnApp::ScheduleNextWaypoint, this);
}

void SingleRouteDtnApp::ScheduleNextWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();
    m_nextWaypointIndex = (m_nextWaypointIndex + 1) % SIRA::route.size();
    point2D target = SIRA::points[SIRA::route[m_nextWaypointIndex]];

    point2D relative = { target.x - currentPos.x,
                         target.y - currentPos.y };
    double dist = relative.length();
    double timeToReach = dist / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRouteDtnApp::ScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRouteDtnApp::ScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> SingleRouteDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = SIRA::route.size();
    for (uint32_t i = 0; i < len; i++) {
        nodeRoute.push_back(SIRA::nodeList[SIRA::route[(i + m_nextWaypointIndex) % len]]);
    }
    return nodeRoute;
}

std::vector<point2D> SingleRouteDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;
    uint32_t len = SIRA::route.size();
    for (uint32_t i = 0; i < len; i++) {
        waypointRoute.push_back(SIRA::points[SIRA::route[(i + m_nextWaypointIndex) % len]]);
    }
    return waypointRoute;
}


void SingleRouteDtnApp::ReceivePacket(Ptr<Socket> socket) {
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
                Simulator::Schedule(jitter, &SingleRouteDtnApp::SendGroundHello, this, topHeader.GetNodeIP());
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
                Schedule_FerryToGround_Transfer(topHeader.GetNodeIP());
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
                Simulator::Schedule(jitter, &SingleRouteDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
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
            Simulator::Schedule(jitter, &SingleRouteDtnApp::SendBundleAck, this, bundle, topHeader.GetNodeIP());
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
                    Schedule_FerryToGround_Transfer(topHeader.GetNodeIP());
            }
            if (m_nodeType == NODE_TYPE_GROUND) {
                Schedule_GroundToFerry_Transfer(topHeader.GetNodeIP());
            }
            continue;
        }
        if (packetType == MessageTypeHeader::FERRY_ACCEPT_TRANSFER) {
            NS_LOG_UNCOND(
                Simulator::Now().GetSeconds()
                << "s Node " << nodeId[m_myIp.Get()]
                << ": ACCEPT TRANSFER from ferry node " << topHeader.GetNodeIP()
            );
            Schedule_GroundToFerry_Transfer(topHeader.GetNodeIP());
        }
    }

}


#endif // SIMPLE_DTN_APP_H

