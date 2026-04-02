#ifndef SR_PIGEON_V2_DTN_APP_H
#define SR_PIGEON_V2_DTN_APP_H

#include "base-dtn-app.h"
#include "simple-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// Route Prunning Delay Triggered Shortcut

class SingleRoutePigeonV2DtnApp : public BaseDtnApp {
    public:
    SingleRoutePigeonV2DtnApp() : BaseDtnApp() {};
    virtual ~SingleRoutePigeonV2DtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SingleRoutePigeonV2DtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<SingleRoutePigeonV2DtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;

    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;

    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void ScheduleNextWaypoint() override;

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

};

NS_OBJECT_ENSURE_REGISTERED(SingleRoutePigeonV2DtnApp);

#pragma region Mobility

void SingleRoutePigeonV2DtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input
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

    Simulator::Schedule(Seconds(0), &SingleRoutePigeonV2DtnApp::ScheduleNextWaypoint, this);
}

void SingleRoutePigeonV2DtnApp::ScheduleNextWaypoint() {
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

    uint32_t dlval1 = 0; // best deadline sastified if go to next waypoint and follow the best pigeon route
    uint32_t dlval2 = 0; // best deadline sastified if start pigeon route now
    double currentTime = Simulator::Now().GetSeconds();
    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = (m_nextFerryIndex + m_direction + m_ferryRoute.size()) % m_ferryRoute.size();

    point2D nextFerryPos = groundNodePos[m_ferryRoute[m_nextFerryIndex]];
    point2D nextWaypoint = nextFerryPos;

    bool continueFerry = true;
    if (m_buffer.size() > 0) {
        point2D relative = { nextFerryPos.x - currentPos.x,
                             nextFerryPos.y - currentPos.y };
        double distance = relative.length();
        double timeToReach = distance / config.ferrySpeed;
        auto route1 = TSPDeadlineHelper(groundNodePos, deadlines,
            nextFerryPos,
            currentTime + timeToReach,
            config.ferrySpeed,
            config.hoverTime,
            &dlval1,
            50, 500
        );
        auto route2 = TSPDeadlineHelper(groundNodePos, deadlines,
            { currentPos.x, currentPos.y },
            currentTime,
            config.ferrySpeed,
            config.hoverTime,
            &dlval2,
            50, 500
        );
        dlval1 = m_buffer.size() - dlval1;
        dlval2 = m_buffer.size() - dlval2;

        if (dlval2 > 0) {
            uint32_t nextFerryNode = groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get();
            uint32_t firstPigeonNode = groundNodeIps[route2[0]].Get();

            if (firstPigeonNode != nextFerryNode) {
                double dist1 = distance; // distance to next ferry node
                double dist2 = dist(groundNodePos[route2[0]], { currentPos.x, currentPos.y }); // distance to first pigeon route node
                double value1 = (double)(dlval1 + 1) / dist1;
                double value2 = (double)(dlval2 + 1) / dist2;

                if (config.SR_PIGEON_V2_addlvt) {
                    double maxdl = std::max(dlval1, dlval2);
                    double timeValue1 = 1;
                    if (m_visitTime.find(nextFerryNode) != m_visitTime.end()) {
                        uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[nextFerryNode];
                        double timeFromLastVisit = (double)time / 1000000.0;
                        timeValue1 = timeFromLastVisit / Simulator::Now().GetSeconds();
                    }
                    double timeValue2 = 1;
                    if (m_visitTime.find(firstPigeonNode) != m_visitTime.end()) {
                        uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[firstPigeonNode];
                        double timeFromLastVisit = (double)time / 1000000.0;
                        timeValue2 = timeFromLastVisit / Simulator::Now().GetSeconds();
                    }
                    value1 = (double)(dlval1 / maxdl + timeValue1) / dist1;
                    value2 = (double)(dlval2 / maxdl + timeValue2) / dist2;
                }


                if (config.waypointSelectMode == DETERMINISTIC) {
                    if (value1 >= value2) {
                        nextWaypoint = nextFerryPos;
                    }
                    else {
                        nextWaypoint = groundNodePos[route2[0]];
                        continueFerry = false;
                        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
                            if (m_ferryRoute[i] == route2[0]) {
                                m_nextFerryIndex = i;
                                break;
                            }
                        }
                    }
                }
                else if (config.waypointSelectMode == PROBABILISTIC) {
                    double randVal = m_rand->GetValue(0.0, value1 + value2);
                    if (randVal < value1) {
                        nextWaypoint = nextFerryPos;
                    }
                    else {
                        nextWaypoint = groundNodePos[route2[0]];
                        continueFerry = false;
                        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
                            if (m_ferryRoute[i] == route2[0]) {
                                m_nextFerryIndex = i;
                                break;
                            }
                        }
                    }
                }
                else {
                    NS_LOG_UNCOND("FATAL: Unknown waypoint select mode " << config.waypointSelectMode);
                    NS_ASSERT_MSG(false, "Unknown waypoint select mode");
                }
            }
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
    point2D relative = { nextWaypoint.x - currentPos.x,
                        nextWaypoint.y - currentPos.y };
    double distance = relative.length();
    double timeToReach = distance / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRoutePigeonV2DtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRoutePigeonV2DtnApp::HoverAndScheduleNextWaypoint, this);

    if (config.SR_PIGEON_V2_vtModePredict) { // predict the visit time of the target node to share with other uavs
        SetVisitTime(
            groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get(),
            Simulator::Now().GetMicroSeconds() + timeToReach * 1000000.0
        );
    }

    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

}

std::vector<uint32_t> SingleRoutePigeonV2DtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> route;
    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        route.push_back(groundNodeIps[m_ferryRoute[(i * m_direction + m_nextFerryIndex + len) % len]].Get());
    }
    return route;
    // return { groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get() };
}

#pragma endregion
#pragma region Bundle

#pragma endregion
#endif 