#ifndef DIVIDED_ROUTE_COUPLING_DTN_APP_H
#define DIVIDED_ROUTE_COUPLING_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <fstream>

#include "../ferry_helper/graph-helper.h"
#include "../ferry_helper/cluster-helper.h"
#include "../ferry_helper/tsp-helper.h"

#pragma region DRC Setup
namespace DRC {
    typedef std::vector<waypoint2D> DRCRoute;
    struct RendezvousSolution {
        uint32_t idx1;
        uint32_t idx2;
        point2D pos;
        double cost;
        bool operator<(const RendezvousSolution& other) const {
            return cost < other.cost;
        }
    };
    std::vector<std::vector<uint32_t>> m_clusters;
    Graph m_connectGraph;


    DRCRoute CleanupRoute(DRCRoute route) {
        uint32_t n = route.size();

        DRCRoute newRoute;
        for (uint32_t i = 0; i < n; i++) {
            if (!route[i].isRendezvous) {
                newRoute.push_back(route[i]);
                continue;
            }
            bool inserted = false;
            for (auto& newWP : newRoute) {
                if (!newWP.isRendezvous) continue;
                point2D A = newWP.pos;
                point2D B = newRoute[i].pos;
                if (dist(A, B) < config.commRange * 0.1) {
                    newWP.tagList.push_back(route[i].tag);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                newRoute.push_back(route[i]);
                newRoute.back().tagList.push_back(route[i].tag);
            }
        }
        return newRoute;
    }

    std::pair<point2D, double> SampleRendezvous(point2D A, point2D B, point2D C, point2D D, double l1, double l2) {
        l1 -= dist(A, B);
        l2 -= dist(C, D);
        double bestCost = std::numeric_limits<double>::max();
        point2D bestPos;

        point2D P = midPoint(A, B);
        point2D Q = midPoint(C, D);
        std::vector<line2D> sampleLines;;
        sampleLines.push_back({ P, Q });
        sampleLines.push_back({ A, B });
        sampleLines.push_back({ C, D });
        sampleLines.push_back({ A, C });
        sampleLines.push_back({ A, D });
        sampleLines.push_back({ B, C });
        sampleLines.push_back({ B, D });
        for (auto& line : sampleLines) {
            uint32_t sampleCount = std::min(config.DRC_sampleCount, (uint32_t)(dist(line.first, line.second) / 1.0));
            point2D unit = (line.second - line.first) / sampleCount;
            for (uint32_t i = 0; i < sampleCount; i++) {
                point2D M = line.first + unit * i;
                double newL1 = l1 + dist(A, M) + dist(M, B);
                double newL2 = l2 + dist(C, M) + dist(M, D);
                double cost = std::max(newL1, newL2) + 0.05 * (newL1 + newL2);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestPos = M;
                }
            }
        }

        return { bestPos, bestCost };
    }
    std::pair<DRCRoute, DRCRoute> connectRoute(const DRCRoute& route1, const DRCRoute& route2, uint32_t rIndex1, uint32_t rIndex2) {
        auto new_route1 = route1;
        auto new_route2 = route2;

        // remove previously connected waypoint
        for (uint32_t i = 0; i < new_route1.size(); i++)
            if (new_route1[i].isRendezvous && new_route1[i].tag == rIndex2) {
                new_route1.erase(new_route1.begin() + i);
                break;
            }
        for (uint32_t i = 0; i < new_route2.size(); i++)
            if (new_route2[i].isRendezvous && new_route2[i].tag == rIndex1) {
                new_route2.erase(new_route2.begin() + i);
                break;
            }
        // calculate route length
        double L1 = 0;
        double L2 = 0;
        for (uint32_t i = 0; i < new_route1.size(); i++) {
            point2D A = new_route1[i].pos;
            point2D B = new_route1[(i + 1) % new_route1.size()].pos;
            L1 += dist(A, B);
        }
        for (uint32_t i = 0; i < new_route2.size(); i++) {
            point2D A = new_route2[i].pos;
            point2D B = new_route2[(i + 1) % new_route2.size()].pos;
            L2 += dist(A, B);
        }

        // add rendezvous waypoint
        RendezvousSolution best;
        best.cost = std::numeric_limits<double>::max();
        for (uint32_t i = 0; i < new_route1.size(); i++) {
            point2D A = new_route1[i].pos;
            point2D B = new_route1[(i + 1) % new_route1.size()].pos;
            for (uint32_t j = 0; j < new_route2.size(); j++) {
                point2D C = new_route2[j].pos;
                point2D D = new_route2[(j + 1) % new_route2.size()].pos;
                auto rendezvous = SampleRendezvous(A, B, C, D, L1, L2);
                if (rendezvous.second < best.cost) {
                    best = { i, j, rendezvous.first, rendezvous.second };
                }
            }
        }
        new_route1.insert(new_route1.begin() + best.idx1 + 1, { best.pos, rIndex2, true });
        new_route2.insert(new_route2.begin() + best.idx2 + 1, { best.pos, rIndex1, true });
        new_route1 = TwoOpt(new_route1);
        new_route2 = TwoOpt(new_route2);

        return { new_route1, new_route2 };
    }

    std::vector<std::vector<uint32_t>> m_groupDistance;
    std::vector<DRCRoute> m_routes;
    std::vector<double> m_routeLength;
    void FerrySetup(const std::vector<std::vector<uint32_t>>& clusters) {
        algoConfig.sendRouteInHello = false;
        m_clusters = clusters;
        NS_ASSERT_MSG(config.nFerrys == clusters.size(), "FATAL: Number of ferry and cluster must equal");

        NS_LOG_UNCOND("DRC: Building graph..");
        std::vector<point2D> centroid;
        for (auto& cluster : clusters) {
            centroid.push_back(getCentroid(groundNodePos, cluster, { 0,0 }));
        }
        if (config.DRC_graphMode == DRC_ONE_CENTER)
            m_connectGraph = BuildOneCenterGraph(centroid);
        else if (config.DRC_graphMode == DRC_TWO_CENTER)
            m_connectGraph = BuildTwoCenterGraph(centroid);
        else if (config.DRC_graphMode == DRC_GABRIEL)
            m_connectGraph = BuildGabrielGraph(centroid);
        else
            NS_ASSERT_MSG(false, "FATAL: Invalid graph mode " + config.DRC_graphMode);

        NS_LOG_UNCOND("DRC: Calculate initial routes..");
        m_routes.resize(config.nFerrys);
        m_routeLength.resize(config.nFerrys);
        for (uint32_t f = 0; f < config.nFerrys; f++) {
            auto route = TSPHelper(groundNodePos, DataStructureHelper::GetReversedSet(clusters[f], groundNodePos.size()), 100, 2000);
            for (uint32_t i : route) {
                m_routes[f].push_back({ groundNodePos[i], i, false });
            }
        }

        NS_LOG_UNCOND("DRC: Calculate rendezvous routes..");
        std::vector<std::pair<uint32_t, uint32_t>> connectIndexList; // list of ferry pair that will be connected
        for (uint32_t u = 0; u < config.nFerrys; u++) {
            for (uint32_t v : m_connectGraph.adjacent[u]) {
                if (u < v) {
                    connectIndexList.push_back({ u, v });
                }
            }
        }
        for (auto& pair : connectIndexList) {
            uint32_t u = pair.first;
            uint32_t v = pair.second;
            auto newRoutes = connectRoute(m_routes[u], m_routes[v], u, v);
            m_routes[u] = newRoutes.first;
            m_routes[v] = newRoutes.second;
        }
        for (uint32_t i = 0; i <= config.DRC_refineIterations; i++) {
            NS_LOG_UNCOND("DRC: Refinning connections.. " << i);
            for (auto& pair : connectIndexList) {
                uint32_t u = pair.first;
                uint32_t v = pair.second;
                auto newRoutes = connectRoute(m_routes[u], m_routes[v], u, v);
                m_routes[u] = newRoutes.first;
                m_routes[v] = newRoutes.second;
            }
        }
        NS_LOG_UNCOND("DRC: Cleaning up routes..");
        for (uint32_t f = 0; f < config.nFerrys; f++) {
            m_routes[f] = CleanupRoute(m_routes[f]);
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
        NS_LOG_UNCOND("DRC: Done");
        return;
    }

    uint32_t currentFerry = -1;
    DRCRoute AssignRoute() {
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
        for (uint32_t i = 0; i < config.nFerrys; i++) {
            for (auto i : m_connectGraph.adjacent[i]) {
                file << i << " ";
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
class DividedRouteCouplingDtnApp : public BaseDtnApp {
    public:
    DividedRouteCouplingDtnApp() : BaseDtnApp() {};
    virtual ~DividedRouteCouplingDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::DividedRouteCouplingDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<DividedRouteCouplingDtnApp>();
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
    DRC::DRCRoute m_ferryRoute;
    bool m_reachedFirstWaypoint = false;
    std::map<uint32_t, bool> m_metParter;
    void ScheduleNextWaypoint();
};

NS_OBJECT_ENSURE_REGISTERED(DividedRouteCouplingDtnApp);

void DividedRouteCouplingDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {

    m_direction = 1;
    auto exclude = DataStructureHelper::GetReversedSet(servingNodesIndex, groundNodePos.size());
    m_ferryRoute = DRC::AssignRoute();
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (m_ferryRoute[i].isRendezvous) {
            m_nextWaypointIndex = i - m_direction;
            if (m_nextWaypointIndex < 0) m_nextWaypointIndex += m_ferryRoute.size();
            break;
        }
    }
    Simulator::Schedule(Seconds(0), &DividedRouteCouplingDtnApp::ScheduleNextWaypoint, this);
}

void DividedRouteCouplingDtnApp::ScheduleNextWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();

    if (m_reachedFirstWaypoint && m_ferryRoute[m_nextWaypointIndex].isRendezvous) {
        m_reachedFirstWaypoint = true;
        auto currentWaypoint = m_ferryRoute[m_nextWaypointIndex];
        bool wait = false;
        for (auto f : currentWaypoint.tagList) { // xét từng đối tác sẽ gặp tại waypoint này
            if (m_metParter.find(f) != m_metParter.end() && m_metParter[f])
                continue; // đã gặp trong thời gian chờ

            uint32_t ferryId = ferryIps[f].Get();

            if (config.DRC_graphMode == DRC_ONE_CENTER || config.DRC_graphMode == DRC_TWO_CENTER) {
                bool isLeaf = DRC::m_connectGraph.adjacent[m_groupId].size() == 1;
                if (m_neighbor.find(ferryId) == m_neighbor.end()) { // chưa từng gặp
                    wait = true; // không break để có thể kiểm tra các partner khác
                }
                else {
                    double lastContact = (double)m_neighbor[ferryId].lastContactTime / 1000000.0;
                    double currentTime = Simulator::Now().GetSeconds();
                    lastContact = currentTime - lastContact;

                    if (lastContact > config.DRC_lastContactTimeout) { // đã lâu chưa gặp
                        if (isLeaf) {
                            double neighborRouteLength = DRC::m_routeLength[(uint32_t)nodeGroup[ferryId]];
                            double myRouteLength = DRC::m_routeLength[(uint32_t)m_groupId];
                            double nbDistanceTillContact = neighborRouteLength - lastContact * config.ferrySpeed;
                            if (myRouteLength * 1.5 > nbDistanceTillContact) { // có thể chạy 1 vòng nữa không ?
                                wait = true;
                            }
                        }
                        else
                            wait = true; // nếu không phải uav lá (chỉ kết nối với 1 uav khác) thì phải chờ
                    }
                    else {
                        m_metParter[f] = true;
                    }
                }
            }
            else if (config.DRC_graphMode == DRC_GABRIEL) {
                bool isLeaf = DRC::m_connectGraph.adjacent[m_groupId].size() == 1;
                if (m_neighbor.find(ferryId) == m_neighbor.end()) {
                    wait = true;
                }
                else {
                    double lastContact = (double)m_neighbor[ferryId].lastContactTime / 1000000.0;
                    double currentTime = Simulator::Now().GetSeconds();
                    lastContact = currentTime - lastContact;

                    if (lastContact > config.DRC_lastContactTimeout) { // đã lâu chưa gặp
                        if (isLeaf) {
                            double neighborRouteLength = DRC::m_routeLength[(uint32_t)nodeGroup[ferryId]];
                            double myRouteLength = DRC::m_routeLength[(uint32_t)m_groupId];
                            double nbDistanceTillContact = neighborRouteLength - lastContact * config.ferrySpeed;
                            if (myRouteLength * 1.5 > nbDistanceTillContact) { // có thể chạy 1 vòng nữa không ?
                                wait = true;
                            }
                        }
                        else
                            if (DRC::m_routeLength[(uint32_t)nodeGroup[ferryId]] > DRC::m_routeLength[(uint32_t)m_groupId])
                                wait = true;
                    }
                    else {
                        m_metParter[f] = true;
                    }
                }
            }
            else {
                NS_ASSERT_MSG(false, "FATAL: Invalid graph mode " + config.DRC_graphMode);
            }
        }
        if (wait) {
            m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
            Simulator::Schedule(Seconds(2.0), &DividedRouteCouplingDtnApp::ScheduleNextWaypoint, this);
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
        Simulator::Schedule(Seconds(1.0), &DividedRouteCouplingDtnApp::ScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &DividedRouteCouplingDtnApp::ScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> DividedRouteCouplingDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        if (nextWaypoint.isRendezvous) continue;
        nodeRoute.push_back(groundNodeIps[nextWaypoint.tag].Get());
    }
    return nodeRoute;
}

std::vector<point2D> DividedRouteCouplingDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i <= len; i++) {
        auto nextWaypoint = m_ferryRoute[(i * m_direction + m_nextWaypointIndex) % len];
        waypointRoute.push_back(nextWaypoint.pos);
    }
    return waypointRoute;
}

Bundle* DividedRouteCouplingDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    for (auto& bundle : m_buffer) {
        if (bundle.flag_waitingAck == false) {
            uint32_t expect1 = DRC::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
            uint32_t expect2 = DRC::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
            if (expect1 >= expect2) {
                return &bundle;
            }
        }
    }
    return nullptr; // no bundle to send
}
Bundle* DividedRouteCouplingDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (Bundle& bundle : m_buffer) {
        if (bundle.flag_waitingAck) continue;
        uint32_t expect1 = DRC::m_groupDistance[m_groupId][nodeGroup[bundle.destination.Get()]];
        uint32_t expect2 = DRC::m_groupDistance[nodeGroup[neighborIp.Get()]][nodeGroup[bundle.destination.Get()]];
        if (expect1 > expect2) {
            return &bundle;
        }

    }
    return nullptr;
}

#pragma endregion
#endif // SIMPLE_DTN_APP_H