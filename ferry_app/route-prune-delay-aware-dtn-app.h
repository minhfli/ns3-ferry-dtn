#ifndef RPDLAS_DTN_APP_H
#define RPDLAS_DTN_APP_H

#include "base-dtn-app.h"
#include "simple-dtn-app.h"

#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// Route Prunning Delay Triggered Shortcut

namespace RPDLAS {

    std::vector<std::set<uint32_t>> excludeSets;
    uint32_t currentFerryIdx = 0;

    uint32_t pruneAmount = 0;

    void FerrySetup() {
        excludeSets.resize(config.nFerrys);

        if (config.RPDLAS_pruneMode == RPDLAS_PRUNE_HALF) {
            pruneAmount = (config.nFerrys - 1) / 2;
        }
        else if (config.RPDLAS_pruneMode == RPDLAS_PRUNE_ONE_THIRD) {
            if (config.nFerrys > 3)
                pruneAmount = config.nFerrys / 3;
        }
        else if (config.RPDLAS_pruneMode == RPDLAS_PRUNE_MAXIMAL) {
            if (config.nFerrys >= 3) {
                double cmax = (double)(config.nFerrys - 1.0) / 2.0;
                double kmax = (double)config.nGrounds / (double)config.nFerrys * cmax;
                pruneAmount = (uint32_t)kmax - 1;
            }
        }
        else if (config.RPDLAS_pruneMode == RPDLAS_PRUNE_CLUSTER) {
            if (config.nFerrys >= 2) {
                uint32_t clusterSize = config.nGrounds / config.nFerrys;
                pruneAmount = config.nGrounds - 2 * clusterSize;
            }
        }
        else if (config.RPDLAS_pruneMode == RPDLAS_PRUNE_ONE) {
            if (config.nFerrys >= 3)
                pruneAmount = 1;
        }
        if (pruneAmount == 0) return;

        uint32_t totalPrune = pruneAmount * config.nFerrys;
        uint32_t pdiv = totalPrune / config.nGrounds;
        uint32_t pmod = totalPrune % config.nGrounds;
        std::vector<uint32_t> prio(config.nGrounds, 0);
        std::vector<uint32_t> random_prio = DataStructureHelper::GetShuffleIndexVector(config.nGrounds);
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            prio[i] = pdiv;
        }
        for (uint32_t i = 0; i < pmod; i++) {
            prio[random_prio[i]]++;
        }

        for (uint32_t f = 0; f < config.nFerrys; f++) {
            std::set<uint32_t> exclude;
            uint32_t max_prio = (totalPrune - 1) / config.nGrounds + 1;
            std::vector<uint32_t> cannidateIdxs;
            for (uint32_t g = 0; g < config.nGrounds; g++) {
                if (prio[g] == max_prio) {
                    cannidateIdxs.push_back(g);
                }
            }
            if (cannidateIdxs.size() < pruneAmount) {
                for (auto i : cannidateIdxs) {
                    exclude.insert(i);
                }
                cannidateIdxs.clear();
                for (uint32_t g = 0; g < config.nGrounds; g++) {
                    if (prio[g] == max_prio - 1) {
                        cannidateIdxs.push_back(g);
                    }
                }
            }

            std::vector<uint32_t> randIdxIdx = DataStructureHelper::GetShuffleIndexVector(cannidateIdxs.size());
            for (auto idxidx : randIdxIdx) {
                if (exclude.size() == pruneAmount) break;
                exclude.insert(cannidateIdxs[idxidx]);
            }
            for (auto i : exclude) {
                prio[i]--;
            }
            excludeSets[f] = exclude;

        }

        return;
    }
    std::set<uint32_t> AssginExcludeSet() {
        currentFerryIdx++;
        return excludeSets[currentFerryIdx - 1];
    }
};

class RoutePrunningDeadlineAwareShortcutDtnApp : public BaseDtnApp {
    public:
    RoutePrunningDeadlineAwareShortcutDtnApp() : BaseDtnApp() {};
    virtual ~RoutePrunningDeadlineAwareShortcutDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::RoutePrunningDeadlineAwareShortcutDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<RoutePrunningDeadlineAwareShortcutDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;

    protected:
    virtual std::vector<uint32_t> GetServingNodeRoute() override;
    virtual Bundle* GroundSelectBundleToFerry(Ipv4Address neighborIp) override;
    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void ScheduleNextWaypoint() override;

    private:
    uint32_t m_lastFerryIndex; // index in the m_ferryRoute
    uint32_t m_nextFerryIndex; // index in the m_ferryRoute

    std::set<uint32_t> m_excludeIdxs;
    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array

    bool m_reachedFirstNode = false;

    int m_direction;

    void Reroute(uint32_t removeFerryIndex, uint32_t addNodeIp);
};

NS_OBJECT_ENSURE_REGISTERED(RoutePrunningDeadlineAwareShortcutDtnApp);

#pragma region Mobility

void RoutePrunningDeadlineAwareShortcutDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input
    m_mode = MODE_FERRY;
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_excludeIdxs = RPDLAS::AssginExcludeSet();
    m_ferryRoute = TSPHelper(groundNodePos, m_excludeIdxs);

    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = 0;
    double currentDistance = -1;
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        point2D pos = groundNodePos[m_ferryRoute[i]];
        point2D relative = { pos.x - currentPos.x,
                             pos.y - currentPos.y };
        double distance = relative.length();

        if (currentDistance < 0 - 1 || distance < currentDistance) {
            currentDistance = distance;
            m_nextFerryIndex = i;
            m_lastFerryIndex = i;
        }
    }
    m_reachedFirstNode = false;
    Simulator::Schedule(Seconds(0), &RoutePrunningDeadlineAwareShortcutDtnApp::ScheduleNextWaypoint, this);
}

void RoutePrunningDeadlineAwareShortcutDtnApp::ScheduleNextWaypoint() {
    RemoveExpiredBundles();
    auto deadlines = GetDeadlines();
    if (m_reachedFirstNode) {
        uint32_t currentNodeIdx = m_ferryRoute[m_nextFerryIndex];
        deadlines[currentNodeIdx].clear(); // remove so that deadline route wont include current node
    }
    if (config.RPDLAS_operationMode == RPDLAS_NO_REROUTE_COLLECT_INROUTE) {
        for (auto excludeIdx : m_excludeIdxs) {
            deadlines[excludeIdx].clear(); // remove all deadlines outside of current node route
        }
    }

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

    uint32_t currentFerryIndex = m_nextFerryIndex;
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
            50, 200
        );
        auto route2 = TSPDeadlineHelper(groundNodePos, deadlines,
            { currentPos.x, currentPos.y },
            currentTime,
            config.ferrySpeed,
            config.hoverTime,
            &dlval2,
            50, 200
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

                if (config.waypointSelectMode == DETERMINISTIC) {
                    if (value1 >= value2) {
                        nextWaypoint = nextFerryPos;
                    }
                    else {
                        nextWaypoint = groundNodePos[route2[0]];
                        continueFerry = false;
                        m_nextFerryIndex = route2[0]; // temp, m_nextFerryIndex set to global node id, reset in reroute()
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
                        m_nextFerryIndex = route2[0]; // temp, m_nextFerryIndex set to global node index, reset in reroute()
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
        Reroute(currentFerryIndex, m_nextFerryIndex);
    }
    point2D relative = { nextWaypoint.x - currentPos.x,
                        nextWaypoint.y - currentPos.y };
    double distance = relative.length();
    double timeToReach = distance / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &RoutePrunningDeadlineAwareShortcutDtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }
    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &RoutePrunningDeadlineAwareShortcutDtnApp::HoverAndScheduleNextWaypoint, this);

    if (config.SR_PIGEON_V2_vtModePredict) { // predict the visit time of the target node to share with other uavs
        SetVisitTime(
            groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get(),
            Simulator::Now().GetMicroSeconds() + timeToReach * 1000000.0
        );
    }

    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());

}

std::vector<uint32_t> RoutePrunningDeadlineAwareShortcutDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> route;
    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        route.push_back(groundNodeIps[m_ferryRoute[(i * m_direction + m_nextFerryIndex + len) % len]].Get());
    }
    return route;
    // return { groundNodeIps[m_ferryRoute[m_nextFerryIndex]].Get() };
}

void RoutePrunningDeadlineAwareShortcutDtnApp::Reroute(uint32_t removeFerryIndex, uint32_t addNodeIndex) {
    if (m_excludeIdxs.find(addNodeIndex) == m_excludeIdxs.end()) {
        // new node is still in current route
        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
            if (m_ferryRoute[i] == addNodeIndex) {
                m_nextFerryIndex = i;
                break;
            }
        }

        double distance1 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, m_direction);
        double distance2 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, -m_direction);
        if (distance1 > distance2) {
            m_direction = -m_direction;
        }
        return;
    }

    // remove old route
    if (removeFerryIndex == m_lastFerryIndex) {
        m_lastFerryIndex = (m_lastFerryIndex + m_ferryRoute.size() - m_direction) % m_ferryRoute.size();
    }
    uint32_t lastFerryNodeIndex = m_ferryRoute[m_lastFerryIndex];

    m_excludeIdxs.erase(addNodeIndex);
    m_excludeIdxs.insert(m_ferryRoute[removeFerryIndex]);
    m_ferryRoute.erase(m_ferryRoute.begin() + removeFerryIndex);

    // calculate new route
    if (config.RPDLAS_operationMode == RPDLAS_REROUTE_INSERT) {
        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
            if (m_ferryRoute[i] == addNodeIndex) {
                NS_ASSERT(false);
            }
        }
        uint32_t insertIndex = 0;
        double insertGain = -1;
        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
            point2D pos1 = groundNodePos[m_ferryRoute[i]];
            point2D pos2 = groundNodePos[m_ferryRoute[(i + 1) % m_ferryRoute.size()]];
            point2D newpos = groundNodePos[addNodeIndex];
            double dist1 = dist(pos1, newpos);
            double dist2 = dist(pos2, newpos);
            double dist12 = dist(pos1, pos2);
            double gain = dist1 + dist2 - dist12;
            if (insertGain < 0 || gain < insertGain) {
                insertGain = gain;
                insertIndex = i;
            }
        }
        m_ferryRoute.insert(m_ferryRoute.begin() + insertIndex + 1, addNodeIndex);
    }
    if (config.RPDLAS_operationMode == RPDLAS_REROUTE_OPTIMIZED) {
        m_ferryRoute = TSPHelper(groundNodePos, m_excludeIdxs);
    }

    // check re-direction
    for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
        if (m_ferryRoute[i] == lastFerryNodeIndex) {
            m_lastFerryIndex = i;
        }
        if (m_ferryRoute[i] == addNodeIndex) {
            m_nextFerryIndex = i;
        }
    }
    double distance1 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, m_direction);
    double distance2 = CalRouteDistance(groundNodePos, m_ferryRoute, m_lastFerryIndex, m_nextFerryIndex, -m_direction);
    if (distance1 > distance2) {
        m_direction = -m_direction;
    }
}
#pragma endregion
#pragma region Bundle

Bundle* RoutePrunningDeadlineAwareShortcutDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    std::sort(m_buffer.begin(), m_buffer.end(), [](const Bundle& a, const Bundle& b) {
        return a.creationTime > b.creationTime;
    }); // newest bundle first

    if (config.RPDLAS_operationMode == RPDLAS_NO_REROUTE_COLLECT_INROUTE) {
        auto neighbor = m_neighbor[neighborIp.Get()];
        for (auto& bundle : m_buffer) {
            if (bundle.flag_waitingAck == false) {
                for (auto nodeId : neighbor.route) {
                    if (nodeId == bundle.destination.Get()) {
                        return &bundle;
                    }
                }
            }
        }
    }
    else {
        for (auto& bundle : m_buffer) {
            if (bundle.flag_waitingAck == false) {
                return &bundle;
            }
        }
    }

    return nullptr; // no bundle to send
}


#pragma endregion
#endif 