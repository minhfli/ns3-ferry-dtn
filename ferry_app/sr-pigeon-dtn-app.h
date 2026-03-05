#ifndef SR_PIGEON_DTN_APP_H
#define SR_PIGEON_DTN_APP_H

#include "base-dtn-app.h"
#include "simple-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

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

    virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp);

    // virtual void ReceivePacket(Ptr<Socket> socket);

    private:
    uint32_t m_nextFerryIndex;
    uint32_t m_nextPigeonIndex;
    std::vector<uint32_t> m_route;

    std::set<uint32_t> m_excludeNodes; // TODO excluding node from the ferry route
    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array
    std::vector<uint32_t> m_pigeonRoute; // indexes of the ground node pos array


    int m_direction;

    void ScheduleNextWaypoint();
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
            &cost1
        );
        auto route2 = TSPDeadlineHelper(
            groundNodePos,
            deadlines,
            { currentPos.x, currentPos.y },
            currentTime,
            config.ferrySpeed,
            &cost2
        );

        uint32_t firstPigeonNode = groundNodeIps[route2[0]].Get();
        uint32_t nextFerryNode = groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get();

        if (firstPigeonNode != nextFerryNode) {
            double dist1 = distance; // distance to next ferry node
            double value1 = (double)cost1 / dist1;
            double dist2 = dist(groundNodePos[route2[0]], { currentPos.x, currentPos.y }); // distance to first pigeon route node
            double value2 = (double)cost2 / dist2;

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
    }

    point2D relative = { nextWaypoint.x - currentPos.x,
                        nextWaypoint.y - currentPos.y };
    double distance = relative.length();
    double timeToReach = distance / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);

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

Bundle* SingleRoutePigeonDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;
    auto neighbor = m_neighbor[neighborIp.Get()];

    if (m_mode == MODE_FERRY) {
        if (neighbor.operationMode == MODE_FERRY) { //* FERRY to FERRY: Send bundle that can travel faster
            // filter node that neighbor will go to it faster
            // NS_LOG_UNCOND("check1");
            std::set<uint32_t> nodeFilter = GetFasterNeighborWaypoints(neighbor);
            // NS_LOG_UNCOND("check2");

            for (Bundle& bundle : m_buffer) {
                if (bundle.flag_waitingAck) continue;
                if (nodeFilter.find(bundle.destination.Get()) != nodeFilter.end()) {
                    return &bundle;
                }
            }
            // NS_LOG_UNCOND("check3");

        }
        if (neighbor.operationMode == MODE_PIGEON) { //* FERRY to PIGEON: Send bundle that must meet deadline
            std::map<uint32_t, double> neighborExpectedArrival;
            for (uint32_t i = 0; i < neighbor.route.size(); i++) {
                neighborExpectedArrival[neighbor.route[i]] = (double)neighbor.expectedArrival[i] / 1000000.0;
            }
            for (Bundle& bundle : m_buffer) {
                if (bundle.flag_waitingAck) continue;
                if (neighborExpectedArrival.find(bundle.destination.Get()) == neighborExpectedArrival.end())
                    continue;
                double expect = neighborExpectedArrival[bundle.destination.Get()];
                double bundleExpiration = bundle.creationTime + config.bundleTTL;
                bundleExpiration /= 1000000.0;
                if (bundleExpiration + config.minExpectedArrivalDifference < expect) {
                    return &bundle;
                }
            }
        }
    }
    if (m_mode == MODE_PIGEON) {
        if (neighbor.operationMode == MODE_FERRY) { //* PIGEON to FERRY: Send nothing
            return nullptr;
        }
        if (neighbor.operationMode == MODE_PIGEON) { //* PIGEON to PIGEON: Send bundle that can travel faster, only with bundle that cannot meet deadline
            // filter node that neighbor will go to it faster
            std::set<uint32_t> nodeFilter = GetFasterNeighborWaypoints(neighbor);

            for (Bundle& bundle : m_buffer) {
                if (bundle.flag_waitingAck) continue;
                double expect = (double)CalExpectedArrival(bundle.destination.Get(), GetServingNodeRoute(), GetServingExpectedArrival());
                double bundleExpiration = bundle.creationTime + config.bundleTTL;
                bundleExpiration /= 1000000.0;
                if (bundleExpiration + config.minExpectedArrivalDifference < expect) {
                    continue; // only send bundle that cannot meet deadlines
                }
                if (nodeFilter.find(bundle.destination.Get()) != nodeFilter.end()) {
                    return &bundle;
                }

            }
        }
    }
    return nullptr;
}

#pragma endregion
#endif 