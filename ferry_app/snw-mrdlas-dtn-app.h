#ifndef SNW_MRDLAS_DTN_APP_H
#define SNW_MRDLAS_DTN_APP_H

#include "epidemic-base-dtn-app.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// Mutli Route Deadline Triggered Shortcut

namespace SNW_MRDLAS {
    std::vector<TSPSolution> ferryRoutes;

    void FerrySetup() {
        config.replicationBaseDtnAppMode = REP_DTN_SNW;
        // config.waypointSelectMode = DETERMINISTIC;
        config.SnW_binary = true;

        ferryRoutes.resize(config.nGrounds + 1);

        for (uint32_t g = 0; g < config.nGrounds; g++) {
            auto route = TSPHelper(
                groundNodePos,
                (std::set<uint32_t>) {
                g
            },
                200, 5000
            ); // remove one node from route

            uint32_t minIndex = 0;
            double minCost = std::numeric_limits<double>::max();
            for (uint32_t i = 0; i < route.size(); i++) {
                point2D p1 = groundNodePos[route[i]];
                point2D p2 = groundNodePos[route[(i + 1) % route.size()]];
                double dist1 = dist(p1, groundNodePos[g]);
                double dist2 = dist(p2, groundNodePos[g]);
                if (dist1 + dist2 < minCost) {
                    minCost = dist1 + dist2;
                    minIndex = i;
                }
            }
            route.insert(route.begin() + minIndex + 1, g); // insert removed node to route
            ferryRoutes[g].order = route;
            ferryRoutes[g].cost = ComputeTSPCost(route, groundNodePos);
            ferryRoutes[g].rollTo1();
        }
        ferryRoutes[config.nGrounds].order = TSPHelper(groundNodePos, std::set<uint32_t>(), 100, 4000); // one best route
        ferryRoutes[config.nGrounds].cost = ComputeTSPCost(ferryRoutes[config.nGrounds].order, groundNodePos);
        ferryRoutes[config.nGrounds].rollTo1();

        std::sort(ferryRoutes.begin(), ferryRoutes.end());
        std::vector<TSPSolution> finalFerryRoutes;
        for (uint32_t i = 0; i < ferryRoutes.size(); i++) { // remove duplicated routes
            if (i == 0 || !ferryRoutes[i].checkEqual(ferryRoutes[i - 1])) {
                finalFerryRoutes.push_back(ferryRoutes[i]);
                NS_LOG_UNCOND(ferryRoutes[i].cost);
            }
        }
        std::swap(ferryRoutes, finalFerryRoutes);
    }

    uint32_t currentFerry = 0;
    std::vector<uint32_t> AssignRoute() {
        currentFerry = (currentFerry + 1) % ferryRoutes.size();
        if (config.MRDLAS_routeMode == MRDLAS_ONE_ROUTE_EACH)
            return ferryRoutes[currentFerry].order;
        if (config.MRDLAS_routeMode == MRDLAS_ONE_ROUTE_2_FERRY) {
            return ferryRoutes[currentFerry / 2].order;
        }
        return {};

    }
};

class SNW_MRDLAS_DtnApp : public EpidemicBaseDtnApp {
    public:
    SNW_MRDLAS_DtnApp() : EpidemicBaseDtnApp() {};
    virtual ~SNW_MRDLAS_DtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SNW_MRDLAS_DtnApp")
            .SetParent<EpidemicBaseDtnApp>()
            .AddConstructor<SNW_MRDLAS_DtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;

    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual std::vector<std::vector<WeightedDeadline>> GetWeightedDeadlines() override;

    // virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp) override;

    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void ScheduleNextWaypoint() override;

    private:
    uint32_t m_lastFerryIndex;
    uint32_t m_nextFerryIndex;

    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array

    bool m_reachedFirstNode = false;

    int m_direction;

};

NS_OBJECT_ENSURE_REGISTERED(SNW_MRDLAS_DtnApp);

#pragma region Mobility

void SNW_MRDLAS_DtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input

    m_mode = MODE_FERRY;
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_ferryRoute = SNW_MRDLAS::AssignRoute();

    Vector3D currentPos = m_mobility->GetPosition();
    point2D currentPos2D = { currentPos.x, currentPos.y };

    m_nextFerryIndex = 0;
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (dist(groundNodePos[m_ferryRoute[i]], currentPos2D) < dist(groundNodePos[m_nextFerryIndex], currentPos2D)) {
            m_nextFerryIndex = i;
        }
    }

    m_reachedFirstNode = false;

    Simulator::Schedule(Seconds(0), &SNW_MRDLAS_DtnApp::ScheduleNextWaypoint, this);
}

void SNW_MRDLAS_DtnApp::ScheduleNextWaypoint() {
    RemoveExpiredBundles();

    double dlval1 = 0; // best deadline sastified if go to next waypoint and follow the best pigeon route
    double dlval2 = 0; // best deadline sastified if start pigeon route now
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
        std::vector<uint32_t> route1, route2;
        if (config.MRDLAS_weightedDeadline == false) { // all bundle have the same weight of 1
            auto deadlines = GetDeadlines();
            uint32_t cost1, cost2;
            route1 = TSPDeadlineHelper(groundNodePos, deadlines,
               nextFerryPos,
               currentTime + timeToReach,
               config.ferrySpeed,
               config.hoverTime,
               &cost1,
               80, 1000
            );
            route2 = TSPDeadlineHelper(groundNodePos, deadlines,
               { currentPos.x, currentPos.y },
               currentTime,
               config.ferrySpeed,
               config.hoverTime,
               &cost2,
               80, 1000
            );
            dlval1 = m_buffer.size() - cost1;
            dlval2 = m_buffer.size() - cost2;
        }
        else { // a bundle will have a weight of distance between its source and destination node
            auto weightedDeadlines = GetWeightedDeadlines();
            double cost1, cost2;
            route1 = TSPWeightedDeadlineHelper(groundNodePos, weightedDeadlines,
               nextFerryPos,
               currentTime + timeToReach,
               config.ferrySpeed,
               config.hoverTime,
               &cost1,
               80, 1000
            );
            route2 = TSPWeightedDeadlineHelper(groundNodePos, weightedDeadlines,
               { currentPos.x, currentPos.y },
               currentTime,
               config.ferrySpeed,
               config.hoverTime,
               &cost2,
               80, 1000
            );
            dlval1 = -cost1;
            dlval2 = -cost2;
        }
        if (dlval2 > 0) {
            uint32_t nextFerryNode = groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get();
            uint32_t firstPigeonNode = groundNodeIps[route2[0]].Get();

            if (firstPigeonNode != nextFerryNode) {
                double dist1 = distance; // distance to next ferry node
                double dist2 = dist(groundNodePos[route2[0]], { currentPos.x, currentPos.y }); // distance to first pigeon route node
                double value1 = (double)(dlval1 + 1) / dist1;
                double value2 = (double)(dlval2 + 1) / dist2;

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
        Simulator::Schedule(Seconds(1.0), &SNW_MRDLAS_DtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SNW_MRDLAS_DtnApp::HoverAndScheduleNextWaypoint, this);

    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

}

std::vector<uint32_t> SNW_MRDLAS_DtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> route;
    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        route.push_back(groundNodeIps[m_ferryRoute[(i * m_direction + m_nextFerryIndex + len) % len]].Get());
    }
    return route;
    // return { groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get() };
}

std::vector<std::vector<WeightedDeadline>> SNW_MRDLAS_DtnApp::GetWeightedDeadlines() {
    RemoveExpiredBundles();
    std::vector<std::vector<WeightedDeadline>> deadlines;
    deadlines.resize(config.nGrounds);
    for (auto bundle : m_buffer) {
        uint32_t node = rawNodeId(bundle.destination.Get());
        double dl = bundle.creationTime + config.bundleTTL; //microsec
        dl /= 1000000.0;
        deadlines[node].push_back({ bundle.replication, dl });
    }
    return deadlines;
}

#pragma endregion
#pragma region Bundle


#pragma endregion
#endif 