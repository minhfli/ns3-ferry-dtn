#ifndef SR_PIGEON_DTN_APP_H
#define SR_PIGEON_DTN_APP_H

#include "base-dtn-app.h"
#include "simple-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// algo name: Single Route Delay Triggered Shortcut

class SingleRoutePigeonDtnApp : public BaseDtnApp {
    public:
    SingleRoutePigeonDtnApp() : BaseDtnApp() {};
    virtual ~SingleRoutePigeonDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SingleRoutePigeonDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<SingleRoutePigeonDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;

    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;

    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void ScheduleNextWaypoint() override;

    private:
    uint32_t m_nextFerryIndex;
    uint32_t m_nextPigeonIndex;
    std::vector<uint32_t> m_route;

    std::set<uint32_t> m_excludeNodes; // TODO excluding node from the ferry route
    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array
    std::vector<uint32_t> m_pigeonRoute; // indexes of the ground node pos array


    int m_direction;

};

NS_OBJECT_ENSURE_REGISTERED(SingleRoutePigeonDtnApp);

#pragma region Mobility

void SingleRoutePigeonDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input
    if (!SIRA::created) {
        SIRA::createRoute();
    }

    m_mode = MODE_FERRY;
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_ferryRoute = SIRA::route;

    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = SIRA::GetClosestIndex({ currentPos.x, currentPos.y }) - m_direction;

    Simulator::Schedule(Seconds(0), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);
}


void SingleRoutePigeonDtnApp::ScheduleNextWaypoint() {
    RemoveExpiredBundles();
    auto deadlines = GetDeadlines();

    uint32_t cost1 = 0; // best cost if go to next waypoint and follow the best pigeon route
    uint32_t cost2 = 0; // best cost if start pigeon route now
    double currentTime = Simulator::Now().GetSeconds();
    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = (m_nextFerryIndex + m_direction + m_ferryRoute.size()) % m_ferryRoute.size();

    point2D nextFerryPos = groundNodePos[m_ferryRoute[m_nextFerryIndex]];
    point2D nextWaypoint = nextFerryPos;

    if (m_buffer.size() > 0) {

        point2D relative = { nextFerryPos.x - currentPos.x,
                             nextFerryPos.y - currentPos.y };
        double distance = relative.length();
        double timeToReach = distance / config.ferrySpeed;
        auto route1 = TSPDeadlineHelper(
            groundNodePos,
            deadlines,
            nextFerryPos,
            currentTime + timeToReach,
            config.ferrySpeed,
            config.hoverTime,
            &cost1,
            50,
            200
        );
        auto route2 = TSPDeadlineHelper(
            groundNodePos,
            deadlines,
            { currentPos.x, currentPos.y },
            currentTime,
            config.ferrySpeed,
            config.hoverTime,
            &cost2,
            50,
            200
        );

        uint32_t firstPigeonNode = groundNodeIps[route2[0]].Get();
        uint32_t nextFerryNode = groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get();

        if (firstPigeonNode != nextFerryNode) {
            double dist1 = distance; // distance to next ferry node
            double value1 = (double)(cost1 + 1) / dist1;
            double dist2 = dist(groundNodePos[route2[0]], { currentPos.x, currentPos.y }); // distance to first pigeon route node
            double value2 = (double)(cost2 + 1) / dist2;

            if (config.waypointSelectMode == DETERMINISTIC) {
                if (value1 >= value2) {
                    nextWaypoint = nextFerryPos;
                }
                else {
                    nextWaypoint = groundNodePos[route2[0]];
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

    point2D relative = { nextWaypoint.x - currentPos.x,
                        nextWaypoint.y - currentPos.y };
    double distance = relative.length();
    double timeToReach = distance / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRoutePigeonDtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRoutePigeonDtnApp::HoverAndScheduleNextWaypoint, this);

    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}
std::vector<uint32_t> SingleRoutePigeonDtnApp::GetServingNodeRoute() {
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