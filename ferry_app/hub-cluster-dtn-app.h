#ifndef HUB_BASED_CLUSTERING_DTN_APP_H
#define HUB_BASED_CLUSTERING_DTN_APP_H

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
namespace CHUB {

    ClusterSolution m_clusters;
    Graph m_connectGraph;
    std::vector<std::vector<uint32_t>> m_groupDistance;
    std::vector<FerryRoute> m_routes;
    std::vector<double> m_routeLength;
    point2D m_hubPos;


    ClusterSolution GetCluster() {
        if (config.CHUB_virtualHub) {
            m_clusters = Clustering::BalancedMT_wCenterClustering_GA(groundNodePos, config.nFerrys, 200, 5000);
        }
        else { // use 1 uav as real hub
            m_clusters = Clustering::BalancedMT_wCenterClustering_GA(groundNodePos, config.nFerrys - 1, 200, 5000);
            m_clusters.push_back({ });
        }
        Clustering::BMTC_ComputeCost(m_clusters, groundNodePos, &m_hubPos);
        return m_clusters;
    }


    void FerrySetup() {
        algoConfig.sendRouteInHello = false;
        NS_LOG_UNCOND("CHUB: Setting up routes..");
        m_routes.resize(config.nFerrys);
        m_routeLength = std::vector<double>(config.nFerrys, 0);
        for (uint32_t i = 0; i < config.nFerrys; i++) {
            if (m_clusters[i].size() == 0) { // this uav is used as hub
                m_routes[i].push_back({ m_hubPos, 0, true });
                continue;
            }

            m_clusters[i] = TSPHelper(groundNodePos, m_clusters[i], 100, 2000);

            double minInsertCost = std::numeric_limits<double>::max();
            uint32_t minInsertIndex = 0;
            for (uint32_t j = 0; j < m_clusters[i].size(); j++) {
                point2D a = groundNodePos[m_clusters[i][j]];
                point2D b = groundNodePos[m_clusters[i][(j + 1) % m_clusters[i].size()]];
                if (dist(a, m_hubPos) + dist(b, m_hubPos) - dist(a, b) < minInsertCost) {
                    minInsertCost = dist(a, m_hubPos) + dist(b, m_hubPos) - dist(a, b);
                    minInsertIndex = j;
                }
            }
            for (uint32_t j = 0; j < m_clusters[i].size(); j++) {
                m_routes[i].push_back({ groundNodePos[m_clusters[i][j]],m_clusters[i][j], false });
                if (j == minInsertIndex) {
                    // điểm gặp mật tất cả ferry còn lại
                    m_routes[i].push_back({ m_hubPos, 0, true, DataStructureHelper::GetIndexVector(config.nFerrys) });
                }
            }
            m_routes[i] = TwoOpt(m_routes[i]);
        }
        NS_LOG_UNCOND("CHUB: calculating connection distance..");
        if (config.CHUB_virtualHub) {
            m_groupDistance = std::vector<std::vector<uint32_t>>(config.nFerrys, std::vector<uint32_t>(config.nFerrys, 1));
            for (uint32_t i = 0; i < config.nFerrys; i++) {
                m_groupDistance[i][i] = 0;
            }
        }
        else { // use 1 uav as real hub
            m_groupDistance = std::vector<std::vector<uint32_t>>(config.nFerrys, std::vector<uint32_t>(config.nFerrys, 2));
            for (uint32_t i = 0; i < config.nFerrys; i++) {
                m_groupDistance[i][config.nFerrys - 1] = 1; // the hub uav distance to orther hubs is always 1
                m_groupDistance[config.nFerrys - 1][i] = 1; // the hub uav distance to orther hubs is always 1
            }
            for (uint32_t i = 0; i < config.nFerrys - 1; i++) {
                m_groupDistance[i][i] = 0; // the distance to itself is always 0
            }
        }

        NS_LOG_UNCOND("CHUB: calculating route length..");
        for (uint32_t i = 0; i < config.nFerrys; i++) {
            for (uint32_t j = 0; j < m_routes[i].size(); j++) {
                m_routeLength[i] += dist(m_routes[i][j].pos, m_routes[i][(j + 1) % m_routes[i].size()].pos);
            }
        }

    }


    uint32_t currentFerry = -1;
    FerryRoute AssignRoute() {
        currentFerry++;
        NS_ASSERT_MSG(currentFerry < config.nFerrys, "FATAL: Ferry index out of bound");
        return m_routes[currentFerry];
    }

    void LogAdditionalInfo(std::string filename) {
        NS_LOG_UNCOND("CHUB: Logging additional info..");
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
    }
};

#pragma endregion

#pragma region Dtn Application
class HubBasedClusteringDtnApp : public BaseDtnApp {
    public:
    HubBasedClusteringDtnApp() : BaseDtnApp() {};
    virtual ~HubBasedClusteringDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::HubBasedClusteringDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<HubBasedClusteringDtnApp>();
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
    std::set<uint32_t> m_metParter;
    void ScheduleNextWaypoint();
};

NS_OBJECT_ENSURE_REGISTERED(HubBasedClusteringDtnApp);

void HubBasedClusteringDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    m_direction = 1;
    auto exclude = DataStructureHelper::GetReversedSet(servingNodesIndex, groundNodePos.size());
    m_ferryRoute = CHUB::AssignRoute();
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (m_ferryRoute[i].isRendezvous) {
            m_nextWaypointIndex = i - m_direction;
            if (m_nextWaypointIndex < 0) m_nextWaypointIndex += m_ferryRoute.size();
            break;
        }
    }
    Simulator::Schedule(Seconds(0), &HubBasedClusteringDtnApp::ScheduleNextWaypoint, this);
}

void HubBasedClusteringDtnApp::ScheduleNextWaypoint() {

    Vector3D currentPos = m_mobility->GetPosition();
    if (m_reachedFirstWaypoint) {
        if (m_ferryRoute.size() == 1) {
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            return;
        }
    }

    if (config.CHUB_virtualHub && m_reachedFirstWaypoint && m_ferryRoute[m_nextWaypointIndex].isRendezvous) {
        // Chỉ wait với chế độ virtual hub
        m_reachedFirstWaypoint = true;
        auto currentWaypoint = m_ferryRoute[m_nextWaypointIndex];
        bool wait = false;
        for (auto f : currentWaypoint.tagList) { // xét từng đối tác sẽ gặp tại waypoint này
            if (m_metParter.find(f) != m_metParter.end())
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
                    m_metParter.insert(f); // đã gặp
                }
            }
        }
        if (wait) {
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            Simulator::Schedule(Seconds(2.0), &HubBasedClusteringDtnApp::ScheduleNextWaypoint, this);
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
        Simulator::Schedule(Seconds(1.0), &HubBasedClusteringDtnApp::ScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &HubBasedClusteringDtnApp::ScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> HubBasedClusteringDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        if (nextWaypoint.isRendezvous) continue;
        nodeRoute.push_back(groundNodeIps[nextWaypoint.tag].Get());
    }
    return nodeRoute;
}

std::vector<point2D> HubBasedClusteringDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i <= len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        waypointRoute.push_back(nextWaypoint.pos);
    }
    return waypointRoute;
}

Bundle* HubBasedClusteringDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            uint32_t expect1 = CHUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
            uint32_t expect2 = CHUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
            if (expect1 >= expect2) { // sender is ground node can send to uav that serve the ground node
                return &bundle;
            }
        }
    }
    return nullptr; // no bundle to send
}
Bundle* HubBasedClusteringDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (Bundle& bundle : m_buffer) {
        if (bundle.flag_waitingAck) continue;
        uint32_t expect1 = CHUB::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
        uint32_t expect2 = CHUB::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
        if (expect1 > expect2) { // uav only send to other uav that can get closer to target
            return &bundle;
        }

    }
    return nullptr;
}

#pragma endregion
#endif // SIMPLE_DTN_APP_H