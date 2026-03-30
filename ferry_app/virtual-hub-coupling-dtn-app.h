#ifndef VIRTUAL_HUB_DTN_APP_H
#define VIRTUAL_HUB_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <fstream>

#include "../ferry_helper/graph-helper.h"
#include "../ferry_helper/cluster-helper.h"
#include "../ferry_helper/tsp-helper.h"

#pragma region VHUB Setup
namespace VHUB {

    std::vector<std::vector<uint32_t>> m_clusters;
    Graph m_connectGraph;

    std::vector<std::vector<uint32_t>> m_groupDistance;
    std::vector<FerryRoute> m_routes;
    std::vector<double> m_routeLength;

    point2D RefineHubPosition(point2D basePos, std::vector<FerryRoute> refineRoutes, double learningRate, uint32_t iteration) {
        while (iteration > 0) {
            iteration--;
            uint32_t furthestRouteIdx = 0;
            double maxDist = 0;
            for (uint32_t i = 0; i < refineRoutes.size(); i++) {
                FerryRoute route = refineRoutes[i];
                if (route.size() == 0) continue;
                double len = 0;
                double minInsertCost = std::numeric_limits<double>::max();
                for (uint32_t j = 0; j < route.size(); j++) {
                    point2D A = route[j].pos;
                    point2D B = route[(j + 1) % route.size()].pos;
                    len += dist(A, B);
                    minInsertCost = std::min(minInsertCost, dist(basePos, A) + dist(basePos, B) - dist(A, B));
                }
                len += minInsertCost;
                if (len > maxDist) {
                    maxDist = len;
                    furthestRouteIdx = i;
                }
            }
            FerryRoute route = refineRoutes[furthestRouteIdx];
            point2D routeCentroid = { 0 ,0 };
            for (uint32_t j = 0; j < route.size(); j++) {
                routeCentroid = routeCentroid + route[j].pos;
            }
            routeCentroid = routeCentroid / route.size();
            basePos = basePos * (1 - learningRate) + routeCentroid * learningRate;
        }
    }

    void FerrySetup(const std::vector<std::vector<uint32_t>>& clusters) {
        algoConfig.sendRouteInHello = false;
        m_clusters = clusters;
        NS_ASSERT_MSG(config.nFerrys == clusters.size(), "FATAL: Number of ferry and cluster must equal");

        std::vector<point2D> centroid;
        for (auto& cluster : clusters) {
            centroid.push_back(getCentroid(groundNodePos, cluster, { 0,0 }));
        }

        NS_LOG_UNCOND("VHUB: Calculate initial routes..");
        m_routes.resize(config.nFerrys);
        m_routeLength.resize(config.nFerrys);
        for (uint32_t f = 0; f < config.nFerrys; f++) {
            auto route = TSPHelper(groundNodePos, DataStructureHelper::GetReversedSet(clusters[f], groundNodePos.size()), 100, 2000);
            for (uint32_t i : route) {
                m_routes[f].push_back({ groundNodePos[i], i, false });
            }
        }

        NS_LOG_UNCOND("VHUB: Calculating HUB position..");
        point2D hubPos = getCentroid(centroid);
        RefineHubPosition(hubPos, m_routes, 0.1, 10);
        RefineHubPosition(hubPos, m_routes, 0.05, 50);
        RefineHubPosition(hubPos, m_routes, 0.02, 50);
        RefineHubPosition(hubPos, m_routes, 0.01, 50);
        RefineHubPosition(hubPos, m_routes, 0.005, 100);
        RefineHubPosition(hubPos, m_routes, 0.002, 100);
        RefineHubPosition(hubPos, m_routes, 0.001, 100);

        for (uint32_t f = 0; f < config.nFerrys; f++) {
            auto& route = m_routes[f];
            if (route.size() == 0) {
                route.push_back({ hubPos, f, true });
                continue;
            }
            double minInsertCost = std::numeric_limits<double>::max();
            uint32_t insertIdx = 0;
            for (uint32_t i = 0; i < route.size(); i++) {
                point2D A = route[i].pos;
                point2D B = route[(i + 1) % route.size()].pos;
                m_routeLength[f] += dist(A, B);
                double insertCost = dist(hubPos, A) + dist(hubPos, B) - dist(A, B);
                if (insertCost < minInsertCost) {
                    minInsertCost = insertCost;
                    insertIdx = i;
                }
            }
            route.insert(route.begin() + insertIdx + 1, { hubPos, 0, true, DataStructureHelper::GetIndexVector(config.nFerrys) });
            m_routeLength[f] += minInsertCost;
        }


        for (uint32_t f = 0; f < config.nFerrys; f++) {
            m_routeLength[f] = 0;
            for (uint32_t i = 0; i < m_routes[f].size(); i++) {
                point2D A = m_routes[f][i].pos;
                point2D B = m_routes[f][(i + 1) % m_routes[f].size()].pos;
                m_routeLength[f] += dist(A, B);
            }
        }

        NS_LOG_UNCOND("DRC: Calculating group distance..");
        m_groupDistance = std::vector<std::vector<uint32_t>>(config.nFerrys, std::vector<uint32_t>(config.nFerrys, 10000000));
        for (uint32_t u = 0; u < config.nFerrys; u++) {
            for (uint32_t v = 0; v < config.nFerrys; v++)
                m_groupDistance[u][v] = 1;
            m_groupDistance[u][u] = 0;
        }
        // Floyd - Warshaw
        for (uint32_t k = 0; k < config.nFerrys; k++) {
            for (uint32_t i = 0; i < config.nFerrys; i++) {
                for (uint32_t j = 0; j < config.nFerrys; j++) {
                    m_groupDistance[i][j] = std::min(m_groupDistance[i][j], m_groupDistance[i][k] + m_groupDistance[k][j]);
                }
            }
        }
        NS_LOG_UNCOND("DRC: Done");
        return;
    }

    uint32_t currentFerry = -1;
    FerryRoute AssignRoute() {
        currentFerry++;
        NS_ASSERT_MSG(currentFerry < config.nFerrys, "FATAL: Ferry index out of bound");
        return m_routes[currentFerry];
    }

    void LogAdditionalInfo(std::string filename) {
        NS_LOG_UNCOND("DRC: Logging additional info..");
        std::ofstream file;
        file.open(filename);
        file << "G: " << config.nGrounds << "\n";
        file << "F: " << config.nFerrys << "\n";
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
        file.close();
    }
};

#pragma endregion

#pragma region Dtn Application
class VirtualHubDtnApp : public BaseDtnApp {
    public:
    VirtualHubDtnApp() : BaseDtnApp() {};
    virtual ~VirtualHubDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VirtualHubDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<VirtualHubDtnApp>();
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

    private:
    uint32_t m_nextWaypointIndex;
    int m_direction;
    FerryRoute m_ferryRoute;
    bool m_reachedFirstWaypoint = false;
    std::map<uint32_t, bool> m_metParter;
    void ScheduleNextWaypoint();
};

NS_OBJECT_ENSURE_REGISTERED(VirtualHubDtnApp);

void VirtualHubDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {

    m_direction = 1;
    auto exclude = DataStructureHelper::GetReversedSet(servingNodesIndex, groundNodePos.size());
    m_ferryRoute = VHUB::AssignRoute();
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (m_ferryRoute[i].isRendezvous) {
            m_nextWaypointIndex = i - m_direction;
            if (m_nextWaypointIndex < 0) m_nextWaypointIndex += m_ferryRoute.size();
            break;
        }
    }
    Simulator::Schedule(Seconds(0), &VirtualHubDtnApp::ScheduleNextWaypoint, this);
}

void VirtualHubDtnApp::ScheduleNextWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();

    if (m_reachedFirstWaypoint && m_ferryRoute[m_nextWaypointIndex].isRendezvous) {
        m_reachedFirstWaypoint = true;
        auto currentWaypoint = m_ferryRoute[m_nextWaypointIndex];
        bool wait = false;
        for (auto f : currentWaypoint.tagList) { // xét từng đối tác sẽ gặp tại waypoint này
            if (m_metParter.find(f) != m_metParter.end() && m_metParter[f])
                continue; // đã gặp trong thời gian chờ
            if (f == (uint32_t)m_groupId)
                continue;

            uint32_t ferryId = ferryIps[f].Get();

            if (m_neighbor.find(ferryId) == m_neighbor.end()) { // chưa từng gặp
                wait = true; // không break để có thể kiểm tra các partner khác
            }
            else {
                double lastContact = (double)m_neighbor[ferryId].lastContactTime / 1000000.0;
                double currentTime = Simulator::Now().GetSeconds();
                lastContact = currentTime - lastContact;
                if (lastContact > config.DRC_lastContactTimeout) { // đã lâu chưa gặp
                    wait = true;
                }
                else {
                    m_metParter[f] = true;
                }
            }
        }
        if (wait) {
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            Simulator::Schedule(Seconds(2.0), &VirtualHubDtnApp::ScheduleNextWaypoint, this);
            return;
        }
        m_metParter.clear(); // cleanup after leaving this waypoint
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
        Simulator::Schedule(Seconds(1.0), &VirtualHubDtnApp::ScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &VirtualHubDtnApp::ScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> VirtualHubDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        if (nextWaypoint.isRendezvous) continue;
        nodeRoute.push_back(groundNodeIps[nextWaypoint.tag].Get());
    }
    return nodeRoute;
}

std::vector<point2D> VirtualHubDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i <= len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        waypointRoute.push_back(nextWaypoint.pos);
    }
    return waypointRoute;
}

Bundle* VirtualHubDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            uint32_t expect1 = VHUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
            uint32_t expect2 = VHUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
            if (expect1 >= expect2) {
                return &bundle;
            }
        }
    }
    return nullptr; // no bundle to send
}
Bundle* VirtualHubDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (Bundle& bundle : m_buffer) {
        if (bundle.flag_waitingAck) continue;
        uint32_t expect1 = VHUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
        uint32_t expect2 = VHUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
        if (expect1 > expect2) {
            return &bundle;
        }

    }
    return nullptr;
}

#pragma endregion
#endif // SIMPLE_DTN_APP_H