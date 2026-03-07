#ifndef TABA2S_DTN_APP_H
#define TABA2S_DTN_APP_H

#include "base-dtn-app.h"
#include "tabaf-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Trajatory aware, buffer aware, 2 step look ahead
 */

class Taba2sDtnApp : public TabafDtnApp {
    public:
    Taba2sDtnApp() : TabafDtnApp() {};
    virtual ~Taba2sDtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::Taba2sDtnApp")
            .SetParent<TabafDtnApp>()
            .AddConstructor<Taba2sDtnApp>();
        return tid;
    }

    protected:


    virtual void CalculateNodeScore() override;

};

NS_OBJECT_ENSURE_REGISTERED(Taba2sDtnApp);

void Taba2sDtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();
    Vector3D currentPos = m_mobility->GetPosition();
    double currentTime = Simulator::Now().GetSeconds();

    auto deadlines = GetDeadlines();
    double max2stepDeadline = 0;

    for (uint32_t i = 0; i < config.nGrounds; i++) {
        point2D relative1 = { groundNodePos[i].x - currentPos.x,
                              groundNodePos[i].y - currentPos.y };
        double dist1 = relative1.length();
        double TTR1 = dist1 / config.ferrySpeed;

        double count1 = 0;
        for (double d : deadlines[i]) {
            if (d < currentTime + TTR1) {
                count1++;
            }
        }
        m_nodeScore[i] = count1;
        for (uint32_t j = 0; j < config.nGrounds; j++) {
            if (i == j) continue;
            point2D relative2 = { groundNodePos[j].x - groundNodePos[i].x,
                                  groundNodePos[j].y - groundNodePos[i].y };
            double dist2 = relative2.length();
            double TTR2 = dist2 / config.ferrySpeed;
            double count2 = 0;
            for (double d : deadlines[j]) {
                if (d < currentTime + TTR2) {
                    count2++;
                }
            }
            m_nodeScore[i] = std::max(m_nodeScore[i], count1 + count2); // temporary assign
            max2stepDeadline = std::max(max2stepDeadline, m_nodeScore[i]);
        }
    }

    for (uint32_t i = 0; i < config.nGrounds; i++) {
        point2D relative = { groundNodePos[i].x - currentPos.x,
                             groundNodePos[i].y - currentPos.y };
        double dist = relative.length();

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

        double bundleValue = 0;
        if (max2stepDeadline != 0) {
            bundleValue = m_nodeScore[i] / max2stepDeadline;
        }

        m_nodeScore[i] = (timeValue + bundleValue) / dist;
    }
}

#endif 