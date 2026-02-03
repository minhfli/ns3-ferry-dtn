#ifndef VIZ_HELPER_H
#define VIZ_HELPER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"


#include "config.h"
#include "global.h"
#include "packet-helper.h"
#include "datatypes.h"

#include <string>
#include <ostream>
#include <vector>

using namespace ns3;


namespace FerryVisualizer {
    std::string vizFileName = "/mnt/d/coding/python/dtn-visualizer/dtn.log";
    std::fstream file;

    void logPosition();
    void logDeclare();
    void logPacket(const std::string& src, const std::string& dest, const std::string meta);
    void logInit();
    void logBeacon(std::string node);

    void logRoute(const std::string& node, const std::vector<point2D> waypoints);
    std::vector<point2D> tspRouteHelper(const std::vector<point2D>& points, const std::vector<uint32_t>& order, uint32_t startIndex);
    /**
     * @param list list of bundle
     */
    void logBuffer(const std::string& node, const std::vector<Bundle>& list);


    void SetUp() {
        file.open(vizFileName, std::ios::out);
        file << "--Declare" << std::endl;
        logDeclare();
        file << "--Events" << std::endl;
        logInit();
        Simulator::Schedule(Time(0), &logPosition);
    }
    void CleanUp() {
        file.close();
    }

    void logInit() {
        file << "Time=0" << std::endl;
        for (uint32_t i = 0; i < simVar.nGrounds; i++) {
            file << "event=buffer"
                << " node=g" << simVar.groundNodes->Get(i)->GetId()
                << " list=|"
                << " reason=init"
                << std::endl;
        }
        for (uint32_t i = 0; i < simVar.nFerrys; i++) {
            file << "event=buffer"
                << " node=f" << simVar.ferryNodes->Get(i)->GetId()
                << " list=|"
                << " reason=init"
                << std::endl;
        }
    }


    void logDeclare() {
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

    void logPosition() {
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
        Simulator::Schedule(MilliSeconds(config.positionLogInterval), &logPosition);
    }

    void logPacket(const std::string& src, const std::string& dest, const std::string meta) {

        file << "Time=" << Simulator::Now().GetSeconds() << std::endl;
        file << "event=send" << " source=" << src << " dest=" << dest << " meta=" << meta << std::endl;
    }

    void logBuffer(const std::string& node, const std::vector<Bundle>& list) {
        file << "Time=" << Simulator::Now().GetSeconds() << std::endl;
        file << "event=buffer" << " node=" << node << " list=";
        for (auto bundle : list) {
            file << nodeId[bundle.source.Get()] << ":" << nodeId[bundle.destination.Get()] << ":" << bundle.id;
            file << "|";
        }
        file << " reason=update" << std::endl;
    }

    void logRoute(const std::string& node, const std::vector<point2D> waypoints) {
        file << "Time=" << Simulator::Now().GetSeconds() << std::endl;
        file << "event=route" << " node=" << node << " tour=";
        for (auto waypoint : waypoints) {
            file << waypoint.x << ":" << waypoint.y << "|";
        }
        file << std::endl;
    }

    std::vector<point2D> tspRouteHelper(const std::vector<point2D>& points, const std::vector<uint32_t>& order, uint32_t startIndex) {
        int n = order.size();
        std::vector<point2D> route;
        for (int i = 0; i < n; i++) {
            startIndex = (startIndex + 1) % n;
            route.push_back(points[order[startIndex]]);
        }
        return route;
    }

    void logBeacon(std::string node) {
        file << "Time=" << Simulator::Now().GetSeconds() << std::endl;
        file << "event=beacon" << " node=" << node << std::endl;
    }


}

#endif // VIZ_HELPER_H