#ifndef TABAF_DTN_APP_H
#define TABAF_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * This is my NS3 implementation of https://link.springer.com/article/10.1007/s11036-018-1038-7
 */

class TabafDtnApp : public BaseDtnApp {
    public:
    TabafDtnApp() : BaseDtnApp() {};
    virtual ~TabafDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::TabafDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<TabafDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual std::vector<point2D> GetServingWaypointRoute() override;

    protected:

    virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp) override;

    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void BundleAckMobilityCallBack(Bundle b) override;
    virtual void CalculateNodeScore();
    virtual void ChooseNextServingNode();

    private:
    uint32_t m_nextServingNode = 0;
    std::vector<double> m_nodeScore;

    EventId m_mobilityScheduleEvent;

    void ScheduleNextWaypoint();
};

NS_OBJECT_ENSURE_REGISTERED(TabafDtnApp);

void TabafDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    m_nodeScore.resize(servingNodesIndex.size(), 0.0);
    Simulator::Schedule(Seconds(0), &TabafDtnApp::ScheduleNextWaypoint, this);
}

void TabafDtnApp::ScheduleNextWaypoint() {
    m_mobilityScheduleEvent.Cancel();

    if (m_nextServingNode != 0) { // current serving node is a valid node
        SetVisitTime(m_nextServingNode, Simulator::Now().GetMicroSeconds());
    }

    ChooseNextServingNode();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

    point2D waypoint = nodePos(m_nextServingNode);
    Vector3D currentPos = m_mobility->GetPosition();

    point2D relative = { waypoint.x - currentPos.x,
                         waypoint.y - currentPos.y };

    double timeToReach = relative.length() / config.ferrySpeed;
    if (timeToReach < 1.0) {
        timeToReach = 1.0;
        m_mobility->SetVelocity(Vector3D(0.0, 0.0, 0.0));
    }
    else {
        m_mobility->SetVelocity(Vector3D(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    }

    m_mobilityScheduleEvent = Simulator::Schedule(Seconds(timeToReach), &TabafDtnApp::ScheduleNextWaypoint, this);

}

void TabafDtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();

    std::map<uint32_t, uint32_t> bundleCountMap = GetBundleCount();
    uint32_t maxCount = std::max_element(bundleCountMap.begin(), bundleCountMap.end(),
         [](const std::pair<uint32_t, uint32_t>& a, const std::pair<uint32_t, uint32_t>& b) {
             return a.second < b.second;
    })->second;

    for (int i = 0; i < config.nGrounds; i++) {
        Vector3D currentPos = m_mobility->GetPosition();
        point2D relative = { groundNodePos[i].x - currentPos.x,
                             groundNodePos[i].y - currentPos.y };
        double dist = relative.length();

        if (dist < 1) {
            m_nodeScore[i] = 0;
            continue;
        }

        double timeValue = 1;
        if (m_visitTime.find(groundNodeIps[i].Get()) != m_visitTime.end()) {
            uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[groundNodeIps[i].Get()];
            double timeFromLastVisit = (double)time / 1000000.0;
            timeValue = timeFromLastVisit / Simulator::Now().GetSeconds();
        }

        double bundleValue = 0;
        if (maxCount != 0 && bundleCountMap.find(groundNodeIps[i].Get()) != bundleCountMap.end()) {
            bundleValue = (double)bundleCountMap[groundNodeIps[i].Get()] / maxCount;
        }

        m_nodeScore[i] = (timeValue + bundleValue) / dist;
    }
}

void TabafDtnApp::ChooseNextServingNode() {

    if (m_nextServingNode == 0) { // inittialize mobility, chose a random node
        m_nextServingNode = groundNodeIps[m_rand->GetInteger(0, groundNodeIps.size() - 1)].Get();
        return;
    }

    CalculateNodeScore();

    if (config.TABAF_waypointSelectMode == SELECT_MODE_RANDOM_MAXIMUM) {
        double maxScore = *std::max_element(m_nodeScore.begin(), m_nodeScore.end());
        std::vector<uint32_t> validNodes;
        for (int i = 0; i < m_nodeScore.size(); i++) {
            if (m_nodeScore[i] == maxScore) {
                validNodes.push_back(groundNodeIps[i].Get());
            }
        }
        m_nextServingNode = validNodes[m_rand->GetInteger(0, validNodes.size() - 1)];
        return;
    }

    if (config.TABAF_waypointSelectMode == SELECT_MODE_PROBALISTC) {
        double totalScore = std::accumulate(m_nodeScore.begin(), m_nodeScore.end(), 0.0);
        if (totalScore == 0) {
            m_nextServingNode = groundNodeIps[m_rand->GetInteger(0, groundNodeIps.size() - 1)].Get();
            return;
        }

        double randvalue = m_rand->GetValue(0.0, totalScore);
        double currentScore = 0.0;
        for (int i = 0; i < m_nodeScore.size(); i++) {
            currentScore += m_nodeScore[i];
            if (currentScore >= randvalue) {
                m_nextServingNode = groundNodeIps[i].Get();
                return;
            }
        }
    }

    NS_LOG_UNCOND("FATAL: Unknown waypoint select mode " << config.TABAF_waypointSelectMode);
    NS_ASSERT_MSG(false, "Unknown waypoint select mode");
}

void TabafDtnApp::BundleAckMobilityCallBack(Bundle b) {
    if (b.destination.Get() != m_nextServingNode)
        return;
    for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
        if (it->destination.Get() == m_nextServingNode) {
            return;
        }
    }
    ScheduleNextWaypoint();
}

std::vector<uint32_t> TabafDtnApp::GetServingNodeRoute() {
    return std::vector<uint32_t>{ m_nextServingNode };
}

std::vector<point2D> TabafDtnApp::GetServingWaypointRoute() {
    return std::vector<point2D>{ nodePos(m_nextServingNode)};
}

Bundle* TabafDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    if (m_buffer.empty()) return nullptr;

    // filter node that neighbor will go to it faster
    auto neighbor = m_neighbor[neighborIp.Get()];

    uint32_t neighbor_target = neighbor.route[neighbor.route.size() - 1];

    if (neighbor_target != m_nextServingNode) { // 2 node have different target
        for (Bundle& bundle : m_buffer) {
            if (!bundle.flag_waitingAck && bundle.destination.Get() == neighbor_target) {
                return &bundle; // send bundle that can go straight to destination
            }
        }
        point2D neighborTargetPos = nodePos(neighbor_target);
        point2D myTargetPos = nodePos(m_nextServingNode);

        for (Bundle& bundle : m_buffer) {
            // nessesary condition
            if (bundle.flag_waitingAck) continue;
            if (neighbor.bufferState.find(bundle.destination.Get()) == neighbor.bufferState.end())  continue;

            point2D bundlePos = nodePos(bundle.destination.Get());
            // sufficient condition 1
            if (dist(neighborTargetPos, bundlePos) < dist(myTargetPos, bundlePos)) continue;

            Vector3D currentPos = m_mobility->GetPosition();
            point2D relative = { bundlePos.x - currentPos.x,
                                 bundlePos.y - currentPos.y };
            double expect1 = Simulator::Now().GetSeconds() + (relative.length() + dist(myTargetPos, bundlePos)) / config.ferrySpeed;
            double expect2 = (double)neighbor.expectedArrival[neighbor.expectedArrival.size() - 1] / 1000000.0;
            expect2 += dist(neighborTargetPos, bundlePos) / config.ferrySpeed;

            // sufficient condition 2
            if (expect1 - expect2 > config.minExpectedArrivalDifference) {
                return &bundle; // send bundle to node that have better chance to reach
            }
        }
    }
    else { // 2 node going to the same target
        double expect2 = neighbor.expectedArrival[neighbor.expectedArrival.size() - 1];
        expect2 /= 1000000.0;

        Vector3D currentPos = m_mobility->GetPosition();
        point2D relative = { nodePos(m_nextServingNode).x - currentPos.x,
                             nodePos(m_nextServingNode).y - currentPos.y };
        double dist = relative.length();
        double expect1 = Simulator::Now().GetSeconds() + dist / config.ferrySpeed;
        if (expect1 - expect2 > config.minExpectedArrivalDifference) {
            for (Bundle& bundle : m_buffer) {
                if (!bundle.flag_waitingAck && bundle.destination.Get() == m_nextServingNode) {
                    return &bundle; // send bundle
                }
            }
        }
    }
    return nullptr;
}
#endif 