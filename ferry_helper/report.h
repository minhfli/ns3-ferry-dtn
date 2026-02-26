#ifndef REPORT_H
#define REPORT_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "ns3/core-module.h"

using namespace ns3;

namespace Report {
    std::string reportFileName = "/mnt/d/coding/python/dtn-visualizer/report.log";

    uint32_t bundleCount = 0;
    uint32_t bundleReachedDestination = 0;
    uint32_t bundleFowardCount = 0; // number of bundle transfered and must be accepted by the destination node
    uint32_t totalHop = 0;
    uint32_t totalHopReachedDestination = 0;
    Time totalDelay = Time(0);
    uint32_t nodeBundleCount[1000][1000];
    uint32_t nodeBundleReachedDestination[1000][1000];

    uint32_t deadConnection = 0;
    uint32_t lowConnection_1p = 0;
    uint32_t lowConnection_2p = 0;
    uint32_t lowConnection_5p = 0;
    uint32_t lowConnection_10p = 0;
    uint32_t lowConnection_20p = 0;

    void Init() {
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            for (uint32_t j = 0; j < config.nFerrys; j++) {
                nodeBundleCount[i][j] = 0;
                nodeBundleReachedDestination[i][j] = 0;
            }
        }
    }
    void Export() {

        for (uint32_t i = 0; i < config.nGrounds; i++) {
            for (uint32_t j = 0; j < config.nFerrys; j++) {
                double deliverRate = (double)100.0 * nodeBundleReachedDestination[i][j] / nodeBundleCount[i][j];
                if (deliverRate < 1)
                    lowConnection_1p++;
                if (deliverRate < 2)
                    lowConnection_2p++;
                if (deliverRate < 5)
                    lowConnection_5p++;
                if (deliverRate < 10)
                    lowConnection_10p++;
                if (deliverRate < 20)
                    lowConnection_20p++;
                if (nodeBundleReachedDestination[i][j] == 0 && nodeBundleCount[i][j] != 0)
                    deadConnection++;
            }
        }

        std::ofstream file(reportFileName);
        file << "--CONFIG" << std::endl;
        file << "Simulation Time: " << config.simTime << " s" << std::endl;
        file << "Area Width: " << config.areaWidth << " m" << std::endl;
        file << "Bundle Generation Rate: " << config.bundleGenRate << " s/bundle" << std::endl;
        file << "Bundle TTL: " << config.bundleTTL / 1000000 << " seconds" << std::endl;
        file << "Comm Range: " << config.commRange << " m" << std::endl;
        file << "Ferry Height: " << config.ferryHeight << " m" << std::endl;
        file << "Ferry Speed: " << config.ferrySpeed << " m/s" << std::endl;
        file << "Ground Buffer Size: " << config.groundBufferSize << std::endl;
        file << "Ferry Buffer Size: " << config.ferryBufferSize << std::endl;
        file << "Number of Grounds: " << config.nGrounds << std::endl;
        file << "Number of Ferries: " << config.nFerrys << std::endl;
        file << "Enable Ferry Communication: " << config.enableFerryComm << std::endl;
        // file << "Waypoint Selection Mode: " << config.waypointSelectMode << std::endl;

        file << "--REPORT" << std::endl;
        file << "Bundle Count: " << bundleCount << std::endl;
        file << "Bundle Reached Destination: " << bundleReachedDestination << std::endl;
        file << "Total Delay: " << totalDelay.GetSeconds() << " seconds" << std::endl;
        file << "Average Delay: " << totalDelay.GetSeconds() / bundleCount << " seconds" << std::endl;
        if (bundleReachedDestination == 0)
            file << "Average Hops: 0" << std::endl;
        else
            file << "Average Hops: " << (double)totalHopReachedDestination / bundleReachedDestination << std::endl;
        file << "Delivery Ratio: " << (double)bundleReachedDestination / bundleCount << std::endl;
        file << "Foward Ratio: " << (double)bundleFowardCount / bundleCount << std::endl;
        file << "Dead Connection: " << deadConnection << std::endl;
        file << "Low Connection (1%): " << lowConnection_1p << std::endl;
        file << "Low Connection (2%): " << lowConnection_2p << std::endl;
        file << "Low Connection (5%): " << lowConnection_5p << std::endl;
        file << "Low Connection (10%): " << lowConnection_10p << std::endl;
        file << "Low Connection (20%): " << lowConnection_20p << std::endl;
        file.close();
    }
}

#endif // REPORT_H