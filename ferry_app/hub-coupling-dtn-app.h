#ifndef HUB_ROUTE_COUPLING_DTN_APP_H
#define HUB_ROUTE_COUPLING_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <fstream>

#include "../ferry_helper/graph-helper.h"
#include "../ferry_helper/cluster-helper.h"
#include "../ferry_helper/tsp-helper.h"

#pragma region Setup
namespace HUB {

    std::vector<std::vector<uint32_t>> m_clusters;
    Graph m_connectGraph;

    std::vector<std::vector<uint32_t>> m_groupDistance;
    std::vector<std::vector<uint32_t>> m_hubs;
    std::vector<FerryRoute> m_routes;
    std::vector<double> m_routeLength;
    std::vector<point2D> m_hubPos;

    void RefineHubPosition(double learningRate, int iteration) {
        while (iteration-- > 0) {
            for (uint32_t h = 0; h < m_hubs.size(); h++) {
                point2D& hPos = m_hubPos[h];

                // Tìm route có lộ trình dài nhất đi qua vị trí hub này
                double maxRouteLength = 0;
                uint32_t maxRouteIndex = 0;
                for (auto i : m_hubs[h]) {
                    FerryRoute& route = m_routes[i];
                    double len = 0;
                    for (uint32_t j = 0; j < route.size(); j++) {
                        point2D A = route[j].pos;
                        point2D B = route[(j + 1) % route.size()].pos;
                        len += dist(A, B);
                        if (len > maxRouteLength) {
                            maxRouteLength = len;
                            maxRouteIndex = i;
                        }
                    }
                }
                // Loại bỏ hub ra khỏi mọi route
                for (auto i : m_hubs[h]) {
                    FerryRoute& route = m_routes[i];
                    for (uint32_t j = 0; j < route.size(); j++) {
                        if (route[j].isRendezvous && route[j].tag == h) {
                            route.erase(route.begin() + j);
                            break;
                        }
                    }
                }
                // Lấy ngẫu nhiên 1 nút của route dài nhất
                point2D target = m_routes[maxRouteIndex][rand() % m_routes[maxRouteIndex].size()].pos;

                // cập nhật hubPos
                hPos = hPos + (target - hPos) * learningRate;
                // thêm lại hPos vào các route
                for (auto i : m_hubs[h]) {
                    FerryRoute& route = m_routes[i];
                    route.push_back({ hPos, h, true });
                    route = TwoOpt(route);
                }
            }
        }
    }

    void FerrySetup(const std::vector<std::vector<uint32_t>>& clusters) {
        algoConfig.sendRouteInHello = false;
        m_clusters = clusters;
        NS_ASSERT_MSG(config.nFerrys == clusters.size(), "FATAL: Number of ferry and cluster must equal");

        NS_LOG_UNCOND("HUB: Building graph..");
        std::vector<point2D> centroid;
        for (auto& cluster : clusters) {
            if (cluster.size() > 0)
                centroid.push_back(getCentroid(groundNodePos, cluster, { 0,0 }));
        }
        // NS_ASSERT_MSG(config.HUB_nHubs == 1, "FATAL: Not implemented yet");

        m_connectGraph = BuildGabrielGraph(centroid);
        m_hubs = GraphBasedSoft2CliqueCluster(m_connectGraph, config.HUB_nHubs);
        m_connectGraph = BuildGraphFromHubs(m_hubs, config.nFerrys, config.nFerrys - config.HUB_nHubs);

        NS_ASSERT_MSG(m_hubs.size() == config.HUB_nHubs, "FATAL: HUB - edge case not implemented");

        NS_LOG_UNCOND("HUB: Calculate initial routes..");
        m_routes = std::vector<FerryRoute>(config.nFerrys, FerryRoute());
        m_routeLength = std::vector<double>(config.nFerrys, 0);
        for (uint32_t f = 0; f < config.nFerrys - config.HUB_nHubs; f++) {
            auto route = TSPHelper(groundNodePos, clusters[f], 100, 2000);
            for (uint32_t i : route) {
                m_routes[f].push_back({ groundNodePos[i], i, false });
            }
        }

        NS_LOG_UNCOND("HUB: Calculating HUB position..");
        for (uint32_t h = 0; h < config.HUB_nHubs; h++) {

            point2D hubCentroid = { 0 ,0 };
            for (uint32_t i = 0; i < m_hubs[h].size(); i++) {
                FerryRoute& route = m_routes[m_hubs[h][i]];
                point2D routeCentroid = { 0 ,0 };
                for (uint32_t j = 0; j < route.size(); j++) {
                    routeCentroid = routeCentroid + route[j].pos;
                }
                routeCentroid = routeCentroid / route.size();
                hubCentroid = hubCentroid + routeCentroid;
            }
            m_hubPos.push_back(hubCentroid);
            for (uint32_t i = 0; i < m_hubs[h].size(); i++) {
                FerryRoute& route = m_routes[m_hubs[h][i]];
                route.push_back({ m_hubPos[h], h, true });
                route = TwoOpt(route);
            }
        }

        RefineHubPosition(0.001, 100);
        RefineHubPosition(0.0001, 1000);

        for (uint32_t h = 0; h < config.HUB_nHubs; h++) { // route setup for hubs
            FerryRoute route;
            route.push_back({ m_hubPos[h], h, true });
            m_routes[config.nFerrys - config.HUB_nHubs + h] = route;
        }

        NS_ASSERT_MSG(m_routes.size() == config.nFerrys, "FATAL: ");

        NS_LOG_UNCOND("HUB: Calculating group distance..");
        m_groupDistance = std::vector<std::vector<uint32_t>>(config.nFerrys, std::vector<uint32_t>(config.nFerrys, 10000000));
        for (uint32_t u = 0; u < config.nFerrys; u++) {
            m_groupDistance[u][u] = 0;
            for (auto adj : m_connectGraph.adjacent[u]) {
                m_groupDistance[u][adj] = 1;
            }
        }
        // Floyd - Warshaw
        for (uint32_t k = 0; k < config.nFerrys; k++) {
            for (uint32_t i = 0; i < config.nFerrys; i++) {
                for (uint32_t j = 0; j < config.nFerrys; j++) {
                    m_groupDistance[i][j] = std::min(m_groupDistance[i][j], m_groupDistance[i][k] + m_groupDistance[k][j]);
                }
            }
        }
        NS_LOG_UNCOND("HUB: Done");
        return;
    }

    uint32_t currentFerry = -1;
    FerryRoute AssignRoute() {
        currentFerry++;
        NS_ASSERT_MSG(currentFerry < config.nFerrys, "FATAL: Ferry index out of bound");
        return m_routes[currentFerry];
    }

    void LogAdditionalInfo(std::string filename) {
        NS_LOG_UNCOND("HUB: Logging additional info..");
        std::ofstream file;
        file.open(filename);
        file << "G: " << config.nGrounds << "\n";
        file << "F: " << config.nFerrys << "\n";
        file << "H: " << config.HUB_nHubs << "\n";
        for (auto p : groundNodePos)
            file << p.x << " " << p.y << " ";
        file << "\n";
        for (auto c : m_clusters) {
            for (auto g : c) {
                file << g << " ";
            }
            file << "\n";
        }
        for (auto r : m_routes) {
            for (auto p : r) {
                file << p.pos.x << " " << p.pos.y << " ";
            }
            file << "\n";
        }
    }
};

#pragma endregion

#pragma region Dtn Application
class HubDtnApp : public BaseDtnApp {
    public:
    HubDtnApp() : BaseDtnApp() {};
    virtual ~HubDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::HubDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<HubDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;


    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual std::vector<point2D> GetServingWaypointRoute() override;
    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual Bundle* GroundSelectBundleToFerry(Ipv4Address neighborIp) override;
    // virtual Bundle* FerrySelectBundleToGround(Ipv4Address neighborIp);
    virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp) override;
    virtual void ScheduleNextWaypoint() override;

    private:
    uint32_t m_nextWaypointIndex;
    int m_direction;
    FerryRoute m_ferryRoute;
    bool m_reachedFirstWaypoint = false;

};

NS_OBJECT_ENSURE_REGISTERED(HubDtnApp);

void HubDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    m_direction = 1;
    auto exclude = DataStructureHelper::GetReversedSet(servingNodesIndex, groundNodePos.size());
    m_ferryRoute = HUB::AssignRoute();
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (m_ferryRoute[i].isRendezvous) {
            m_nextWaypointIndex = i - m_direction;
            if (m_nextWaypointIndex < 0) m_nextWaypointIndex += m_ferryRoute.size();
            break;
        }
    }
    Simulator::Schedule(Seconds(0), &HubDtnApp::ScheduleNextWaypoint, this);
}

void HubDtnApp::ScheduleNextWaypoint() {

    Vector3D currentPos = m_mobility->GetPosition();
    if (m_reachedFirstWaypoint) {
        if (m_ferryRoute.size() == 1) {
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            return;
        }
    }
    m_reachedFirstWaypoint = true;
    m_nextWaypointIndex = (m_nextWaypointIndex + m_direction + m_ferryRoute.size()) % m_ferryRoute.size();
    point2D target = m_ferryRoute[m_nextWaypointIndex].pos;

    point2D relative = { target.x - currentPos.x,
                         target.y - currentPos.y };
    double dist = relative.length();
    double timeToReach = dist / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &HubDtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &HubDtnApp::HoverAndScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> HubDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        if (nextWaypoint.isRendezvous) continue;
        nodeRoute.push_back(groundNodeIps[nextWaypoint.tag].Get());
    }
    return nodeRoute;
}

std::vector<point2D> HubDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i <= len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        waypointRoute.push_back(nextWaypoint.pos);
    }
    return waypointRoute;
}

Bundle* HubDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            uint32_t expect1 = HUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
            uint32_t expect2 = HUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
            if (expect1 >= expect2) { // sender is ground node can send to uav that serve the ground node
                return &bundle;
            }
        }
    }
    return nullptr; // no bundle to send
}
Bundle* HubDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (Bundle& bundle : m_buffer) {
        if (bundle.flag_waitingAck) continue;
        uint32_t expect1 = HUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
        uint32_t expect2 = HUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
        if (expect1 > expect2) { // uav only send to other uav that can get closer to target
            return &bundle;
        }

    }
    return nullptr;
}

#pragma endregion
#endif // SIMPLE_DTN_APP_H