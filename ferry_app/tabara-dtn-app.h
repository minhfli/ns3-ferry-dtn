#ifndef TABARA_DTN_APP_H
#define TABARA_DTN_APP_H

#include "base-dtn-app.h"
#include "tabaf-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Trajatory aware, buffer aware, rate aware ferry
 */

class TabaraDtnApp : public TabafDtnApp {
    public:
    TabaraDtnApp() : TabafDtnApp() {};
    virtual ~TabaraDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::TabaraDtnApp")
            .SetParent<TabafDtnApp>()
            .AddConstructor<TabaraDtnApp>();
        return tid;
    }

    protected:


    virtual void CalculateNodeScore() override;

};

NS_OBJECT_ENSURE_REGISTERED(TabaraDtnApp);

void TabaraDtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();

    std::map<uint32_t, uint32_t> bundleCountMap = GetBundleCount();

    for (uint32_t i = 0; i < config.nGrounds; i++) {
        Vector3D currentPos = m_mobility->GetPosition();
        point2D relative = { groundNodePos[i].x - currentPos.x,
                             groundNodePos[i].y - currentPos.y };
        double dist = relative.length();

        if (dist < 1) {
            m_nodeScore[i] = 0;
            continue;
        }

        double timeFromLastVisit = config.bundleTTL;
        if (m_visitTime.find(groundNodeIps[i].Get()) != m_visitTime.end()) {
            uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[groundNodeIps[i].Get()];
            timeFromLastVisit = time;
        }
        double timeValue = (timeFromLastVisit / 1000000.0 + dist / config.ferrySpeed) * nodeGenRate[groundNodeIps[i].Get()];

        double bundleValue = 0;
        if (bundleCountMap.find(groundNodeIps[i].Get()) != bundleCountMap.end()) {
            bundleValue = (double)bundleCountMap[groundNodeIps[i].Get()];
        }

        m_nodeScore[i] = (timeValue + bundleValue) / dist;
    }
}

#endif 