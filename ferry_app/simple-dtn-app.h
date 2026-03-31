#ifndef SIMPLE_DTN_APP_H
#define SIMPLE_DTN_APP_H

#include "base-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace SIRA {
    bool created = false;
    std::vector<uint32_t> route;
    std::vector<point2D> points;
    std::vector<uint32_t> nodeList;
    void createRoute() {
        points = groundNodePos;
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            nodeList.push_back(groundNodeIps[i].Get());
        }
        if (created) return;
        created = true;
        route = TSPClassicGA(points);
        route = TSPTwoOptOptimize(points, route);
    }

    uint32_t GetClosestIndex(const point2D target, const std::set<uint32_t>& excludeIndex = {}) {
        if (!created) {
            NS_ASSERT_MSG(false, "SIRA route not created");
        }
        uint32_t minIndex = 0;
        while (excludeIndex.find(minIndex) != excludeIndex.end()) {
            minIndex++;
        }
        for (uint32_t i : route) {
            if (excludeIndex.find(i) != excludeIndex.end()) {
                continue;
            }
            if (dist(points[i], target) < dist(points[minIndex], target)) {
                minIndex = i;
            }
        }
        return minIndex;
    }
};

class SingleRouteDtnApp : public BaseDtnApp {
    public:
    SingleRouteDtnApp() : BaseDtnApp() {};
    virtual ~SingleRouteDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SingleRouteDtnApp")
            .SetParent<BaseDtnApp>()
            .AddConstructor<SingleRouteDtnApp>();
        return tid;
    }
    virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;
    virtual std::vector<uint32_t> GetServingNodeRoute() override;

    protected:

    // virtual void ReceivePacket(Ptr<Socket> socket);
    virtual void ScheduleNextWaypoint() override;

    private:
    uint32_t m_nextWaypointIndex;
    int m_direction;
};

NS_OBJECT_ENSURE_REGISTERED(SingleRouteDtnApp);

void SingleRouteDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) {
    if (!SIRA::created) {
        SIRA::createRoute();
    }
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_nextWaypointIndex = m_rand->GetInteger(0, SIRA::route.size() - 1);
    Simulator::Schedule(Seconds(0), &SingleRouteDtnApp::ScheduleNextWaypoint, this);
}

void SingleRouteDtnApp::ScheduleNextWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();
    m_nextWaypointIndex = (m_nextWaypointIndex + m_direction + SIRA::route.size()) % SIRA::route.size();
    point2D target = SIRA::points[SIRA::route[m_nextWaypointIndex]];

    point2D relative = { target.x - currentPos.x,
                         target.y - currentPos.y };
    double dist = relative.length();
    double timeToReach = dist / config.ferrySpeed;

    if (timeToReach < 0.1) {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1.0), &SingleRouteDtnApp::HoverAndScheduleNextWaypoint, this);
        return;
    }

    m_mobility->SetVelocity(Vector(relative.x / timeToReach, relative.y / timeToReach, 0.0));
    Simulator::Schedule(Seconds(timeToReach), &SingleRouteDtnApp::HoverAndScheduleNextWaypoint, this);
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
}

std::vector<uint32_t> SingleRouteDtnApp::GetServingNodeRoute() {
    std::vector<uint32_t> nodeRoute;

    uint32_t len = SIRA::route.size();
    for (uint32_t i = 0; i < len; i++) {
        nodeRoute.push_back(SIRA::nodeList[SIRA::route[(i * m_direction + m_nextWaypointIndex + len) % len]]);
    }
    return nodeRoute;
}

#endif // SIMPLE_DTN_APP_H