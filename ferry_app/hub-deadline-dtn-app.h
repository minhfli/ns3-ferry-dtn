#ifndef HUB_DEADLINE_DTN_APP_H
#define HUB_DEADLINE_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <fstream>

#include "../ferry_helper/graph-helper.h"
#include "../ferry_helper/cluster-helper.h"
#include "../ferry_helper/tsp-helper.h"
#include "../ferry_helper/mtsp-helper.h"

#pragma region Setup
namespace DHUB {

    std::vector<FerryRoute> m_routes;
    std::vector<double> m_nodeScore;
    double m_maxRouteLength;
    double m_minRouteLengthCap;
    double m_scheduledMeetingTime;
    point2D m_hubPos;
    std::vector<double> m_visitTime;
    std::vector<BaseDtnApp*> m_apps;


    void refineHub(const std::vector<point2D>& points, point2D& hub, uint32_t iterations, double learningRate) {
        while (iterations--) {
            point2D furthestPoint = hub;
            for (auto& point : points) {
                double distance = dist(hub, point);
                if (distance > dist(furthestPoint, hub)) {
                    furthestPoint = point;
                }
            }
            hub = hub * (1 - learningRate) + furthestPoint * learningRate;
        }
    }
    void handleNewLoop();

    void FerrySetup() {
        algoConfig.sendRouteInHello = false;

        m_hubPos = getCentroid(groundNodePos);
        refineHub(groundNodePos, m_hubPos, 100, 0.001);
        refineHub(groundNodePos, m_hubPos, 1000, 0.0001);
        refineHub(groundNodePos, m_hubPos, 1000, 0.00001);
        std::vector<double> routeLengths;

        for (auto& point : groundNodePos) {
            routeLengths.push_back(2 * dist(m_hubPos, point) + 2 * config.ferrySpeed * config.hoverTime);
            m_maxRouteLength = std::max(m_maxRouteLength, routeLengths.back());
        }
        std::sort(routeLengths.begin(), routeLengths.end());
        m_minRouteLengthCap = routeLengths[config.nFerrys - 1];

        m_maxRouteLength = std::max(m_maxRouteLength, (double)config.bundleTTL / 1000000.0 * config.ferrySpeed / 2.5);
        m_scheduledMeetingTime = config.warmupTime;

        m_visitTime.resize(config.nGrounds, 0);
        m_nodeScore.resize(config.nGrounds, 0);
        m_routes.resize(config.nFerrys, {});

        Simulator::Schedule(Seconds(m_scheduledMeetingTime - config.hoverTime), &handleNewLoop);
    }


    void delayedSetNewScheduleMeetingTime(double meetingTime) {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " meeting scheduled at " << meetingTime);
        m_scheduledMeetingTime = meetingTime;
        Simulator::Schedule(Seconds(m_scheduledMeetingTime - config.hoverTime) - Simulator::Now(), &handleNewLoop);

    }

    void handleNewLoop() {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " DHUB starting new loop");
        //* prepare
        std::vector<std::vector<double>> deadlines(config.nGrounds);
        for (auto app : m_apps) {
            auto uav_deadlines = app->GetDeadlines();
            for (uint32_t i = 0; i < config.nGrounds; i++) {
                deadlines[i].insert(deadlines[i].end(), uav_deadlines[i].begin(), uav_deadlines[i].end());
            }
        }
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            if (deadlines[i].empty()) continue;
            std::sort(deadlines[i].begin(), deadlines[i].end());
        }

        std::vector<double> timeFromLastVisit;
        for (uint32_t i = 0; i < config.nGrounds; i++)
            timeFromLastVisit.push_back(m_scheduledMeetingTime - m_visitTime[i]);

        //* calculate node score
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            m_nodeScore[i] = deadlines[i].size();
        }
        double maxScore = *std::max_element(m_nodeScore.begin(), m_nodeScore.end()) + 1;
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            m_nodeScore[i] = m_nodeScore[i] / maxScore;
            m_nodeScore[i] += timeFromLastVisit[i] / m_scheduledMeetingTime;
        }
        maxScore = *std::max_element(m_nodeScore.begin(), m_nodeScore.end()) - 0.1;
        double routecap;
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            if (m_nodeScore[i] > maxScore) {
                routecap = 2 * dist(m_hubPos, groundNodePos[i]) + 2 * config.ferrySpeed * config.hoverTime;
                break;
            }
        }
        routecap = std::max(routecap, m_minRouteLengthCap);

        //* calculate new route
        auto clusters = mTSP_cap_dl_vt(
            groundNodePos, config.nFerrys, m_hubPos,
            m_scheduledMeetingTime,
            routecap,
            deadlines,
            timeFromLastVisit,
            500,
            7000
        );

        //* apply new route and reschedule
        double localMaxRouteLength = 0;
        m_routes.resize(config.nFerrys, {});
        for (uint32_t f = 0; f < config.nFerrys; f++) {
            auto& route = m_routes[f];
            route.clear();
            route.push_back({ m_hubPos, 0, true });// add hubpos
            for (auto node : clusters[f]) {
                route.push_back({ groundNodePos[node], node, false });
            }
            localMaxRouteLength = std::max(localMaxRouteLength, GetFerryRouteLength(route, config.hoverTime, config.ferrySpeed));
        }

        Simulator::Schedule(Seconds(config.hoverTime + 1), &delayedSetNewScheduleMeetingTime,
            m_scheduledMeetingTime + localMaxRouteLength / config.ferrySpeed + 0.1);

        return;
    }


};

#pragma endregion

#pragma region Dtn Application
class HubDeadlineClusteringDtnApp : public BaseDtnApp {
    public:
    HubDeadlineClusteringDtnApp() : BaseDtnApp() {};
    virtual ~HubDeadlineClusteringDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::HubDeadlineClusteringDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<HubDeadlineClusteringDtnApp>();
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
    uint32_t m_ferryIndex;
    uint32_t m_nextWaypointIndex;
    FerryRoute m_ferryRoute;
    bool m_reachedFirstWaypoint = false;

};

NS_OBJECT_ENSURE_REGISTERED(HubDeadlineClusteringDtnApp);

void HubDeadlineClusteringDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    m_nextWaypointIndex = 0;
    m_ferryIndex = nodeIndex[m_myIp.Get()];
    m_ferryRoute = { {DHUB::m_hubPos, 0 , true} };
    // m_ferryRoute = DHUB::m_routes[m_ferryIndex];
    Simulator::Schedule(Seconds(0), &HubDeadlineClusteringDtnApp::ScheduleNextWaypoint, this);
}

void HubDeadlineClusteringDtnApp::ScheduleNextWaypoint() {

    Vector3D currentPos = m_mobility->GetPosition();

    point2D target = DHUB::m_hubPos;
    if (m_reachedFirstWaypoint) {
        if (m_ferryRoute[m_nextWaypointIndex].isRendezvous) {
            double currentTime = Simulator::Now().GetSeconds();
            if (currentTime < DHUB::m_scheduledMeetingTime - 0.01) {
                m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
                Simulator::Schedule(Seconds(DHUB::m_scheduledMeetingTime - currentTime), &HubDeadlineClusteringDtnApp::ScheduleNextWaypoint, this);
                return;
            }
            m_ferryRoute = DHUB::m_routes[m_ferryIndex]; // reassign route
        }
        else {
            DHUB::m_visitTime[m_ferryRoute[m_nextWaypointIndex].tag] = Simulator::Now().GetSeconds();
        }
        m_nextWaypointIndex = (m_nextWaypointIndex + 1) % m_ferryRoute.size();
        target = m_ferryRoute[m_nextWaypointIndex].pos;
    }
    m_reachedFirstWaypoint = true;

    point2D relative = { target.x - currentPos.x,
                         target.y - currentPos.y };
    double dist = relative.length();
    double timeToReach = dist / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &HubDeadlineClusteringDtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &HubDeadlineClusteringDtnApp::HoverAndScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> HubDeadlineClusteringDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i < len; i++) {
        auto nextWaypoint = m_ferryRoute[(i + m_nextWaypointIndex) % len];
        if (nextWaypoint.isRendezvous) continue;
        nodeRoute.push_back(groundNodeIps[nextWaypoint.tag].Get());
    }
    return nodeRoute;
}

std::vector<point2D> HubDeadlineClusteringDtnApp::GetServingWaypointRoute() {
    std::vector<point2D> waypointRoute;

    uint32_t len = m_ferryRoute.size();
    for (uint32_t i = 0; i <= len; i++) {
        auto nextWaypoint = m_ferryRoute[(i + m_nextWaypointIndex) % len];
        waypointRoute.push_back(nextWaypoint.pos);
    }
    return waypointRoute;
}

Bundle* HubDeadlineClusteringDtnApp::GroundSelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;
    for (auto& b : m_buffer) {
        if (b.flag_waitingAck == false) {
            return &b;
        }
    }
    return nullptr;
}
Bundle* HubDeadlineClusteringDtnApp::FerrySelectBundleToFerry(Ipv4Address neighborIp) {
    RemoveExpiredBundles();
    if (m_buffer.empty()) return nullptr;

    for (auto& b : m_buffer) {
        if (b.flag_waitingAck) continue;
        bool routeHaveBundleDest = false;
        for (auto& wp : m_ferryRoute) {
            if (wp.isRendezvous) continue;
            if (wp.tag == rawNodeId(b.destination.Get()));
            routeHaveBundleDest = true;
            break;
        }
        if (routeHaveBundleDest) continue;

        uint32_t neighborIndex = nodeIndex[neighborIp.Get()];
        for (auto& wp : DHUB::m_routes[neighborIndex])
            if (wp.tag == rawNodeId(b.destination.Get())) return &b;
    }
    return nullptr;
}

#pragma endregion
#endif // SIMPLE_DTN_APP_H