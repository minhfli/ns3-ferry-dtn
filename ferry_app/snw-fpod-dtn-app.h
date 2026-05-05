#ifndef FPOD_DTN_APP_H
#define FPOD_DTN_APP_H

#include "snw-tabaf-dtn-app.h"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * This is my NS3 implementation of https://link.springer.com/article/10.1007/s11036-018-1038-7
 */

class SNW_FPOD_DtnApp : public SNW_TABAF_DtnApp {
    public:
    SNW_FPOD_DtnApp() : SNW_TABAF_DtnApp() {};
    virtual ~SNW_FPOD_DtnApp() {};

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SNW_FPOD_DtnApp")
            .SetParent<SNW_TABAF_DtnApp>()
            .AddConstructor<SNW_FPOD_DtnApp>();
        return tid;
    }
    // virtual void InitializeMobility(const std::vector<uint32_t>& servingNodesIndex) override;
    // virtual std::vector<uint32_t> GetServingNodeRoute() override;
    // virtual std::vector<point2D> GetServingWaypointRoute() override;

    protected:
    // virtual void ScheduleNextWaypoint() override;

    // virtual Bundle* FerrySelectBundleToFerry(Ipv4Address neighborIp) override;

    // virtual void ReceivePacket(Ptr<Socket> socket);
    // virtual void BundleAckMobilityCallBack(Bundle b) override;
    virtual void CalculateNodeScore();
    // virtual void ChooseNextServingNode();

    // uint32_t m_nextServingNode = 0;
    // std::vector<double> m_nodeScore;

    // EventId m_mobilityScheduleEvent;

};

NS_OBJECT_ENSURE_REGISTERED(SNW_FPOD_DtnApp);

void SNW_FPOD_DtnApp::CalculateNodeScore() {
    RemoveExpiredBundles();

    for (uint32_t i = 0; i < config.nGrounds; i++)
        m_nodeScore[i] = 0;

    for (Bundle& bundle : m_buffer) {
        if (config.TABAF_weightedDeadline) {
            m_nodeScore[rawNodeId(bundle.destination.Get())] +=
                dist(nodePos(bundle.source.Get()), nodePos(bundle.destination.Get()));
        }
        else {
            m_nodeScore[rawNodeId(bundle.destination.Get())] += 1; // temporary set node score to count of bundles to it
        }
    }
    double maxBundleValue = *std::max_element(m_nodeScore.begin(), m_nodeScore.end());

    for (uint32_t i = 0; i < config.nGrounds; i++) {
        Vector3D currentPos = m_mobility->GetPosition();
        point2D relative = { groundNodePos[i].x - currentPos.x,
                             groundNodePos[i].y - currentPos.y };
        double dist = relative.length();

        if (dist < 1) {
            m_nodeScore[i] = 0;
            continue;
        }

        double timeValue = 1;
        if (m_visitTime.find(groundNodeIps[i].Get()) == m_visitTime.end())
            m_visitTime[groundNodeIps[i].Get()] = 0;

        uint64_t time = Simulator::Now().GetMicroSeconds() - m_visitTime[groundNodeIps[i].Get()];
        double timeFromLastVisit = (double)time / 1000000.0;
        timeValue = (timeFromLastVisit + dist / config.ferrySpeed)
            * nodeGenRate[groundNodeIps[i].Get()];

        double bundleValue = m_nodeScore[i];

        m_nodeScore[i] = (timeValue + bundleValue) / dist;
    }
}
#endif 