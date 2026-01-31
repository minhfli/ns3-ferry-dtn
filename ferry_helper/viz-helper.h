#ifndef VIZ_HELPER_H
#define VIZ_HELPER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"


#include "config.h"

#include <string>
#include <ostream>

using namespace ns3;


namespace FerryVisualizer {
    std::string vizFileName = "/home/minh/ferry-viz/ferry.log";
    std::fstream file;

    void updatePosition();
    void declare();
    void logPacket();


    void SetUp() {
        file.open(vizFileName, std::ios::out);
        file << "--Declare" << std::endl;
        declare();
        file << "--Events" << std::endl;
        Simulator::Schedule(Time(0), &updatePosition);
    }
    void CleanUp() {
        file.close();
    }

    void declare() {
        file << "area=" << config.areaWidth << "|" << config.areaWidth << std::endl;
        for (uint32_t i = 0; i < simVar.nGrounds; i++) {
            file << "node=g" << i
                << " type=ground"
                << " group=0"
                << " color=255|0|0"
                << " buffer=" << config.groundBufferSize
                << std::endl;
        }
        // ferry to ground communication range
        double fg_commrange = config.commRange * config.commRange - config.ferryHeight * config.ferryHeight;
        fg_commrange = std::sqrt(fg_commrange);

        for (uint32_t i = 0; i < simVar.nFerrys; i++) {
            file << "node=f" << simVar.ferryNodes->Get(i)->GetId()
                << " type=ferry"
                << " group=0"
                << " color=0|0|255"
                << " buffer=" << config.ferryBufferSize
                << " range=" << config.commRange << "|" << fg_commrange
                << std::endl;
        }
    }


    void updatePosition() {
        file << "Time=" << Simulator::Now().GetSeconds() << std::endl;
        // open file
        if (Simulator::Now() == 0) {
            for (uint32_t i = 0; i < simVar.nGrounds; i++) {
                auto node = simVar.groundNodes->Get(i);
                auto mobility = node->GetObject<MobilityModel>();
                auto pos = mobility->GetPosition();
                file << "event=pos" << " node=g" << node->GetId() << " x=" << pos.x << " y=" << pos.y << std::endl;
            }
        }
        for (uint32_t i = 0; i < simVar.nFerrys; i++) {
            auto node = simVar.ferryNodes->Get(i);
            auto mobility = node->GetObject<MobilityModel>();
            auto pos = mobility->GetPosition();
            file << "event=pos" << " node=f" << node->GetId() << " x=" << pos.x << " y=" << pos.y << std::endl;
        }
        Simulator::Schedule(MilliSeconds(config.positionLogInterval), &updatePosition);
    }
}

#endif // VIZ_HELPER_H