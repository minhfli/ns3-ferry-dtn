#ifndef TABADLA_DTN_APP_H
#define TABADLA_DTN_APP_H

#include "base-dtn-app.h"
#include "tabaf-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Trajatory aware, buffer aware, deadline aware ferry
 */

class TabaDlaDtnApp : public TabafDtnApp {
    public:
    TabaDlaDtnApp() : TabafDtnApp() {};
    virtual ~TabaDlaDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::TabaDlaDtnApp")
            .SetParent<TabafDtnApp>()
            .AddConstructor<TabaDlaDtnApp>();
        return tid;
    }

    protected:

    virtual void CalculateNodeScore();

};

NS_OBJECT_ENSURE_REGISTERED(TabaDlaDtnApp);

void TabaDlaDtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();

    // std::map<uint32_t, uint32_t> bundleCountMap = GetBundleCount();
    // uint32_t maxCount = std::max_element(bundleCountMap.begin(), bundleCountMap.end(),
    //      [](const std::pair<uint32_t, uint32_t>& a, const std::pair<uint32_t, uint32_t>& b) {
    //          return a.second < b.second;
    // })->second;
    double currentTime = Simulator::Now().GetSeconds();
    Vector3D currentPos = m_mobility->GetPosition();

    auto deadlines = GetDeadlines();
    uint32_t maxPossibleDeadline = 0;
    if (m_buffer.size() > 0) {
        TSPDeadlineHelper(groundNodePos, deadlines,
            { currentPos.x, currentPos.y },
            currentTime,
            config.ferrySpeed,
            &maxPossibleDeadline,
            50,
            100
        );
    }

    for (uint32_t i = 0; i < config.nGrounds; i++) {

        point2D relative = { groundNodePos[i].x - currentPos.x,
                             groundNodePos[i].y - currentPos.y };
        double dist = relative.length();
        double timeToReach = dist / config.ferrySpeed;

        if (dist < 1) {
            m_nodeScore[i] = 0;
            continue;
        }

        double timeValue = 1;
        if (m_visitTime.find(groundNodeIps[i].Get()) != m_visitTime.end()) {
            uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[groundNodeIps[i].Get()];
            double timeFromLastVisit = (double)time / 1000000.0;
            timeValue = timeFromLastVisit / Simulator::Now().GetSeconds();
        }

        // double bundleValue = 0;
        // if (config.TABADLA_addBundleValue && maxCount != 0 && bundleCountMap.find(groundNodeIps[i].Get()) != bundleCountMap.end()) {
        //     bundleValue = (double)bundleCountMap[groundNodeIps[i].Get()] / maxCount;
        // }

        double deadlineValue = 0;
        if (maxPossibleDeadline > 0) {
            uint32_t bestTSPDeadlineCost;
            auto route = TSPDeadlineHelper(groundNodePos, deadlines,
                groundNodePos[i],
                currentTime + timeToReach,
                config.ferrySpeed,
                &bestTSPDeadlineCost,
                50,
                100
            );
            deadlineValue = (double)bestTSPDeadlineCost / (double)maxPossibleDeadline;
        }

        // m_nodeScore[i] = (timeValue + bundleValue + deadlineValue) / dist;
        m_nodeScore[i] = (timeValue + deadlineValue) / dist;
    }
}

#endif 