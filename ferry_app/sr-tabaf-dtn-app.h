#ifndef SR_TABAF_DTN_APP_H
#define SR_TABAF_DTN_APP_H

#include "base-dtn-app.h"
#include "simple-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// Route Prunning Delay Triggered Shortcut

class SingleRouteTabafDtnApp : public BaseDtnApp {
    public:
    SingleRouteTabafDtnApp() : BaseDtnApp() {};
    virtual ~SingleRouteTabafDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SingleRouteTabafDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<SingleRouteTabafDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;

    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;

    // virtual void ReceivePacket(Ptr<Socket> socket);

    private:
    uint32_t m_lastFerryIndex;
    uint32_t m_nextFerryIndex;
    uint32_t m_nextPigeonIndex;
    std::vector<uint32_t> m_route;

    std::set<uint32_t> m_excludeNodes; // TODO excluding node from the ferry route
    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array
    std::vector<uint32_t> m_pigeonRoute; // indexes of the ground node pos array

    bool m_reachedFirstNode = false;

    int m_direction;

    std::vector<double> m_nodeScore;

    void ScheduleNextWaypoint();
    void CalculateNodeScore();
};

NS_OBJECT_ENSURE_REGISTERED(SingleRouteTabafDtnApp);

#pragma region Mobility

void SingleRouteTabafDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input
    if (!SIRA::created) {
        SIRA::createRoute();
    }

    m_mode = MODE_FERRY;
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_ferryRoute = SIRA::route;

    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = SIRA::GetClosestIndex({ currentPos.x, currentPos.y }) - m_direction;
    m_reachedFirstNode = false;
    m_nodeScore.resize(config.nGrounds, 0);

    Simulator::Schedule(Seconds(0), &SingleRouteTabafDtnApp::ScheduleNextWaypoint, this);
}

void SingleRouteTabafDtnApp::ScheduleNextWaypoint() {
    RemoveExpiredBundles();
    auto deadlines = GetDeadlines();

    if (!config.SR_PIGEON_V2_vtModePredict) {
        if (m_reachedFirstNode) {
            SetVisitTime(
                groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get(),
                Simulator::Now().GetMicroSeconds()
            );
        }
        else {
            m_reachedFirstNode = true;
        }
    }
    CalculateNodeScore();

    double dlval1 = 0; // score if go to next ferry waypoint 
    double dlval2 = 0; // best score 


    m_nextFerryIndex = (m_nextFerryIndex + m_direction + m_ferryRoute.size()) % m_ferryRoute.size();

    bool continueFerry = true;
    if (m_buffer.size() > 0) {
        dlval1 = m_nodeScore[m_ferryRoute[m_nextFerryIndex]];
        uint32_t skipIndex = 0;
        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
            if (m_nodeScore[m_ferryRoute[i]] < m_nodeScore[m_ferryRoute[skipIndex]]) {
                skipIndex = i;
            }
        }
        dlval2 = m_nodeScore[m_ferryRoute[skipIndex]];

        if (config.waypointSelectMode == DETERMINISTIC) {
            if (dlval1 < dlval2) {
                continueFerry = false;
                m_nextFerryIndex = skipIndex;
            }
        }
        else if (config.waypointSelectMode == PROBABILISTIC) {
            double randVal = m_rand->GetValue(0.0, dlval1 + dlval2);
            if (randVal >= dlval1) {
                continueFerry = false;
                m_nextFerryIndex = skipIndex;
            }
        }
        else {
            NS_LOG_UNCOND("FATAL: Unknown waypoint select mode " << config.waypointSelectMode);
            NS_ASSERT_MSG(false, "Unknown waypoint select mode");
        }
    }

    if (continueFerry) {
        m_lastFerryIndex = m_nextFerryIndex;
    }
    else {
        double distance1 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, m_direction);
        double distance2 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, -m_direction);
        if (distance1 > distance2) {
            m_direction = -m_direction;
        }
    }

    Vector3D currentPos = m_mobility->GetPosition();
    point2D nextWaypoint = groundNodePos[m_ferryRoute[m_nextFerryIndex]];
    point2D relative = { nextWaypoint.x - currentPos.x,
                        nextWaypoint.y - currentPos.y };
    double distance = relative.length();
    double timeToReach = distance / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRouteTabafDtnApp::ScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRouteTabafDtnApp::ScheduleNextWaypoint, this);

    if (config.SR_PIGEON_V2_vtModePredict) { // predict the visit time of the target node to share with other uavs
        SetVisitTime(
            groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get(),
            Simulator::Now().GetMicroSeconds() + timeToReach * 1000000.0
        );
    }

    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

}

std::vector<uint32_t> SingleRouteTabafDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> route;
    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        route.push_back(groundNodeIps[m_ferryRoute[(i * m_direction + m_nextFerryIndex + len) % len]].Get());
    }
    return route;
    // return { groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get() };
}

void SingleRouteTabafDtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();

    std::map<uint32_t, uint32_t> bundleCountMap = GetBundleCount();
    uint32_t maxCount = std::max_element(bundleCountMap.begin(), bundleCountMap.end(),
         [](const std::pair<uint32_t, uint32_t>& a, const std::pair<uint32_t, uint32_t>& b) {
             return a.second < b.second;
    })->second;

    for (uint32_t i = 0; i < config.nGrounds; i++) {
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
#pragma endregion
#pragma region Bundle

#pragma endregion
#endif 