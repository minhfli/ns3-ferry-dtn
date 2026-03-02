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
    virtual std::vector<uint32_t> GetServingNodeRoute() override;

    protected:

    // virtual void ReceivePacket(Ptr<Socket> socket);

    private:
    uint32_t m_mode;
    uint32_t m_nextFerryIndex;
    uint32_t m_nextPigeonIndex;
    std::vector<uint32_t> m_route;

    std::set<uint32_t> m_excludeNodes; // TODO excluding node from the ferry route
    std::vector<uint32_t> m_ferryRoute; // indexes of the ground node pos array
    std::vector<uint32_t> m_pigeonRoute; // indexes of the ground node pos array


    int m_direction;

    void ScheduleNextWaypoint();
    void ScheduleFerryWaypoint();
    void SchedulePigeonWaypoint();
    void CheckModeSwitch();
};

NS_OBJECT_ENSURE_REGISTERED(SingleRoutePigeonDtnApp);

void SingleRoutePigeonDtnApp::InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) { // ignore the input
    if (!SIRA::created) {
        SIRA::createRoute();
    }

    m_mode = MODE_FERRY;
    m_direction = m_rand->GetInteger(0, 1);
    m_direction = 2 * m_direction - 1; // -1 or 1
    m_ferryRoute = SIRA::route;

    Vector3D currentPos = m_mobility->GetPosition();

    m_nextFerryIndex = SIRA::GetClosestIndex({ currentPos.x, currentPos.y });

    Simulator::Schedule(Seconds(0), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);
}


void SingleRoutePigeonDtnApp::CheckModeSwitch() {
    Vector3D currentPos = m_mobility->GetPosition();

    point2D relative = { SIRA::points[SIRA::route[m_nextFerryIndex]].x - currentPos.x,
                         SIRA::points[SIRA::route[m_nextFerryIndex]].y - currentPos.y };

    double distance = relative.length();
    double time = distance / config.ferrySpeed;

    double currentTime = Simulator::Now().GetSeconds();
    auto deadlines = GetDeadlines();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);

    std::vector<uint32_t> pigeonRoute;
    pigeonRoute = TSPDeadlineBasedGA(
        groundNodePos,
        deadlines,
        SIRA::points[SIRA::route[m_nextFerryIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed
    );
    pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
        groundNodePos,
        deadlines,
        SIRA::points[SIRA::route[m_nextFerryIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed,
        pigeonRoute
    );

    uint32_t cost = ComputeDeadlineCost(
        pigeonRoute,
        groundNodePos,
        deadlines,
        SIRA::points[SIRA::route[m_nextFerryIndex]], // starting pos
        currentTime + time,
        config.ferrySpeed
    );
}

void SingleRoutePigeonDtnApp::ScheduleNextWaypoint() {
    RemoveExpiredBundles();
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
    if (m_mode == MODE_FERRY) {
        ScheduleFerryWaypoint();
    }
    else if (m_mode == MODE_PIGEON) {
        SchedulePigeonWaypoint();
    }
    else {
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
    }
    FerryVisualizer::logRoute(nodeId[m_myIp.Get()], GetServingWaypointRoute());
    FerryVisualizer::logBuffer(nodeId[m_myIp.Get()], m_buffer);
}

void SingleRoutePigeonDtnApp::ScheduleFerryWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();
    point2D target = SIRA::points[SIRA::route[m_nextFerryIndex]];
    point2D relative = { target.x - currentPos.x,
                         target.y - currentPos.y };
    double timeToReach = relative.length() / config.ferrySpeed;

    if (!m_buffer.empty()) { // deadline check
        double currentTime = Simulator::Now().GetSeconds();
        auto deadlines = GetDeadlines();
        uint32_t cost;

        m_pigeonRoute = TSPDeadlineBasedGA(
            groundNodePos, deadlines,
            groundNodePos[m_ferryRoute[m_nextFerryIndex]], // starting pos
            currentTime + timeToReach,
            config.ferrySpeed
        );
        m_pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
            groundNodePos, deadlines,
            groundNodePos[m_ferryRoute[m_nextFerryIndex]], // starting pos
            currentTime + timeToReach,
            config.ferrySpeed,
            m_pigeonRoute,
            &cost
        );
        // cannot sastify all deadlines or buffer full -> switch to pigeon mode
        if (cost < m_buffer.size() || m_buffer.size() >= m_maxBufferSize) {
            m_pigeonRoute = TSPDeadlineBasedGA(
                groundNodePos, deadlines,
                { currentPos.x, currentPos.y }, // starting pos
                currentTime,
                config.ferrySpeed
            );
            m_pigeonRoute = TSPDeadlineBasedTwoOptOptimize(
                groundNodePos,
                deadlines,
                { currentPos.x, currentPos.y }, // starting pos
                currentTime,
                config.ferrySpeed,
                m_pigeonRoute
            );
            m_mode = MODE_PIGEON;
            m_nextPigeonIndex = 0;

            SchedulePigeonWaypoint();
            NS_LOG_UNCOND("Switch to pigeon mode");
            return;
        }
    }
    // set velocity
    if (timeToReach < 0.1) {
        Simulator::Schedule(Seconds(config.mobilityWaitTime), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        return;
    }
    Simulator::Schedule(Seconds(timeToReach), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);

    point2D velocity;
    velocity.x = relative.x / timeToReach;
    velocity.y = relative.y / timeToReach;

    m_mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));

    // update target
    m_nextFerryIndex = (m_nextFerryIndex + m_direction + m_ferryRoute.size()) % m_ferryRoute.size();
}

void SingleRoutePigeonDtnApp::SchedulePigeonWaypoint() {
    Vector3D currentPos = m_mobility->GetPosition();
    while (m_nextPigeonIndex < m_pigeonRoute.size()) { // clean up the pigeon route
        uint32_t nodeIdx = m_pigeonRoute[m_nextPigeonIndex];
        uint32_t nodeIp = groundNodeIps[nodeIdx].Get();
        bool noBundleToDeliver = true;
        for (Bundle b : m_buffer) {
            if (b.destination.Get() == nodeIp) {
                noBundleToDeliver = false;
                break;
            }
        }
        if (noBundleToDeliver) {
            m_nextPigeonIndex++; // skip the next node in route
        }
        else {
            break;
        }
    }
    if (m_nextPigeonIndex >= m_pigeonRoute.size()) { // return to ferry
        m_mode = MODE_FERRY;
        Vector3D currentPos = m_mobility->GetPosition();
        point2D pos = { currentPos.x, currentPos.y };
        //find closest
        m_nextFerryIndex = 0;
        for (uint32_t i = 0; i < m_ferryRoute.size(); i++) {
            if (dist(pos, groundNodePos[m_ferryRoute[i]]) < dist(pos, groundNodePos[m_ferryRoute[m_nextFerryIndex]])) {
                m_nextFerryIndex = i;
            }
        }
        ScheduleFerryWaypoint();
        // NS_LOG_UNCOND("Switch to ferry mode");
        return;
    }
    point2D relative = { groundNodePos[m_pigeonRoute[m_nextPigeonIndex]].x - currentPos.x,
                         groundNodePos[m_pigeonRoute[m_nextPigeonIndex]].y - currentPos.y };
    double timeToReach = relative.length() / config.ferrySpeed;

    if (timeToReach < 0.1) {
        Simulator::Schedule(Seconds(1.0), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);
        m_mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        return;
    }
    Simulator::Schedule(Seconds(timeToReach), &SingleRoutePigeonDtnApp::ScheduleNextWaypoint, this);

    point2D velocity;
    velocity.x = relative.x / timeToReach;
    velocity.y = relative.y / timeToReach;
    m_mobility->SetVelocity(Vector(velocity.x, velocity.y, 0));

    m_nextPigeonIndex++;
}

std::vector<uint32_t> SingleRoutePigeonDtnApp::GetServingNodeRoute() {
    if (m_mode == MODE_FERRY) {
        uint32_t len = m_ferryRoute.size();
        std::vector<uint32_t> route;
        for (uint32_t i = 0; i < len; i++) {
            route.push_back(groundNodeIps[m_ferryRoute[(m_nextFerryIndex + i * m_direction + len) % len]].Get());
        }
        return route;
    }
    else if (m_mode == MODE_PIGEON) {
        uint32_t len = m_pigeonRoute.size();
        std::vector<uint32_t> route;
        for (uint32_t i = m_nextPigeonIndex; i < len; i++) {
            route.push_back(groundNodeIps[m_pigeonRoute[i]].Get());
        }
        return route;
    }
    else {
        return {};
    }
}

#endif 