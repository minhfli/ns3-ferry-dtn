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

#include "base-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

class PigeonDtnApp : public BaseDtnApp {
    public:
    PigeonDtnApp() {}
    virtual ~PigeonDtnApp() {}

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::PigeonDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<PigeonDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual std::vector<point2D> GetServingWaypointRoute() override;

    protected:

    // --- CORE LOGIC: Nơi bạn sẽ cài thuật toán Routing --)

    void ReceivePacket(Ptr<Socket> socket);

    private:
    std::vector<std::vector<double>> GetDeadlines();
    void CleanUpPigeonRoute(); // remove waypoint from the pigeon route 

    EventId m_mobilityScheduleEvent;
    void ScheduleNextWaypoint();
    void ScheduleFerryWaypoint();
    void SchedulePigeonWaypoint();
    void SchedulePigeonOnBufferFull();

    std::vector<uint32_t> m_servingNodes;
    std::vector<point2D> m_ferryPoints;
    std::vector<uint32_t> m_ferryOrder;
    std::uint32_t m_ferryNextIndex;

    // std::vector<point2D> m_pigeonPoints; // this is the same as ground node list
    std::vector<uint32_t> m_pigeonOrder;
    std::uint32_t m_pigeonNextIndex;
};

NS_OBJECT_ENSURE_REGISTERED(PigeonDtnApp);


void PigeonDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    m_ferryNextIndex = 0;
    if (servingNodesIndex.empty()) {
        m_ferryPoints = { { 0.0,0.0 } };
        m_ferryOrder = { 0 };
    }
    else {
        for (auto index : servingNodesIndex) {
            m_ferryPoints.push_back(groundNodePos[index]);
            m_servingNodes.push_back(groundNodeIps[index].Get());
        }
        auto order = TSPClassicGA(m_ferryPoints);
        order = TSPTwoOptOptimize(m_ferryPoints, order);
        m_ferryOrder = order;


        Vector3D currentPos = m_mobility->GetPosition();
        for (uint32_t i = 1; i < order.size(); i++) {
            point2D relative = { m_ferryPoints[order[i]].x - currentPos.x,
                                 m_ferryPoints[order[i]].y - currentPos.y };
            point2D startIdRelative = { m_ferryPoints[m_ferryNextIndex].x - currentPos.x,
                                        m_ferryPoints[m_ferryNextIndex].y - currentPos.y };
            double dist1 = relative.x * relative.x + relative.y * relative.y;
            double dist2 = startIdRelative.x * startIdRelative.x + startIdRelative.y * startIdRelative.y;
            if (dist1 < dist2) {
                m_ferryNextIndex = i;
            }
        }
    }
    m_mode = MODE_FERRY;
    m_mobilityScheduleEvent = Simulator::Schedule(Time(0), &PigeonDtnApp::ScheduleNextWaypoint, this);
}

std::vector<uint32_t> PigeonDtnApp::GetServingNodeRoute() {

    if (m_mode == MODE_FERRY) {
        uint32_t len = m_ferryOrder.size();
        std::vector<uint32_t> route;
        for (uint32_t i = 0; i < len; i++) {
            route.push_back(m_servingNodes[m_ferryOrder[(m_ferryNextIndex + i) % len]]);
        }
        return route;
    }
    else if (m_mode == MODE_PIGEON) {
        uint32_t len = m_pigeonOrder.size();
        std::vector<uint32_t> route;
        for (uint32_t i = m_pigeonNextIndex; i < len; i++) {
            route.push_back(groundNodeIps[m_pigeonOrder[i]].Get());
        }
        return route;
    }
    else {
        return {};
    }
}

std::vector<point2D> PigeonDtnApp::GetServingWaypointRoute() {

    if (m_mode == MODE_FERRY) {
        uint32_t len = m_ferryOrder.size();
        std::vector<point2D> route;
        for (uint32_t i = 0; i < len; i++) {
            route.push_back(m_ferryPoints[m_ferryOrder[(m_ferryNextIndex + i) % len]]);
        }
        return route;
    }
    else if (m_mode == MODE_PIGEON) {
        uint32_t len = m_pigeonOrder.size();
        std::vector<point2D> route;
        for (uint32_t i = m_pigeonNextIndex; i < len; i++) {
            route.push_back(groundNodePos[m_pigeonOrder[i]]);
        }
        return route;
    }
    else {
        return {};
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
    RemoveExpiredBundles();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
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
    // cannot sastify all deadlines or buffer full -> switch to pigeon mode
    if (cost < m_buffer.size() || m_buffer.size() >= m_maxBufferSize) {
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
        FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

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
    CleanUpPigeonRoute();
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
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

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

void PigeonDtnApp::SchedulePigeonOnBufferFull() { // TODO remove, this function is no longer used
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

void PigeonDtnApp::CleanUpPigeonRoute() {
    uint32_t len = m_pigeonOrder.size();
    auto deadlines = GetDeadlines();

    for (uint32_t i = m_pigeonNextIndex; i < len; i++) {
        uint32_t node = m_pigeonOrder[i];
        if (deadlines[node].empty()) {
            m_pigeonOrder.erase(m_pigeonOrder.begin() + i);
            i--;
            len--;
        }
    }
}

#endif // PIGEON_DTN_APP_H
