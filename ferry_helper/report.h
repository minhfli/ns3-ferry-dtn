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
    std::vector<Time> delayList;
    uint32_t nodeBundleCount[1000][1000];
    uint32_t nodeBundleReachedDestination[1000][1000];

    uint32_t deadConnection = 0;
    uint32_t lowConnection_1p = 0;
    uint32_t lowConnection_2p = 0;
    uint32_t lowConnection_5p = 0;
    uint32_t lowConnection_10p = 0;
    uint32_t lowConnection_20p = 0;
    uint32_t lowConnection_30p = 0;
    uint32_t lowConnection_50p = 0;

    double averageTop5pDelay = 0; // average delay of top 5% of all generated bundles
    double averageTop10pDelay = 0;
    double averageTop20pDelay = 0;
    double averageTop30pDelay = 0;
    double averageTop50pDelay = 0;

    void Init() {
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            for (uint32_t j = 0; j < config.nFerrys; j++) {
                nodeBundleCount[i][j] = 0;
                nodeBundleReachedDestination[i][j] = 0;
            }
        }
        delayList.reserve(10000);
    }
    void Export() {
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            for (uint32_t j = 0; j < config.nGrounds; j++) {
                if (i == j) continue;
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
                if (deliverRate < 30)
                    lowConnection_30p++;
                if (deliverRate < 50)
                    lowConnection_50p++;
                if (nodeBundleReachedDestination[i][j] == 0 && nodeBundleCount[i][j] != 0)
                    deadConnection++;
            }
        }

        std::sort(delayList.begin(), delayList.end());
        uint32_t count5p = bundleCount * 5 / 100;
        uint32_t count10p = bundleCount * 10 / 100;
        uint32_t count20p = bundleCount * 20 / 100;
        uint32_t count30p = bundleCount * 30 / 100;
        uint32_t count50p = bundleCount * 50 / 100;
        for (uint32_t i = 0; i < delayList.size(); i++) {
            if (i < count5p)
                averageTop5pDelay += delayList[i].GetSeconds();
            if (i < count10p)
                averageTop10pDelay += delayList[i].GetSeconds();
            if (i < count20p)
                averageTop20pDelay += delayList[i].GetSeconds();
            if (i < count30p)
                averageTop30pDelay += delayList[i].GetSeconds();
            if (i < count50p)
                averageTop50pDelay += delayList[i].GetSeconds();
        }
        averageTop5pDelay /= count5p;
        averageTop10pDelay /= count10p;
        averageTop20pDelay /= count20p;
        averageTop30pDelay /= count30p;
        averageTop50pDelay /= count50p;
        if (delayList.size() < count5p) averageTop5pDelay = 0;
        if (delayList.size() < count10p) averageTop10pDelay = 0;
        if (delayList.size() < count20p) averageTop20pDelay = 0;
        if (delayList.size() < count30p) averageTop30pDelay = 0;
        if (delayList.size() < count50p) averageTop50pDelay = 0;

        std::ofstream file(reportFileName);
        file << "--CONFIG" << std::endl;
        file << "Random Seed: " << config.randSeed << std::endl;
        file << "Algorithm: " << config.ALGORITHM_NAME << std::endl;
        file << "Simulation Time (s): " << config.simTime << std::endl;
        file << "Area Width (m): " << config.areaWidth << std::endl;
        file << "Bundle Generation Rate (s/bundle): " << config.bundleGenRate << std::endl;
        file << "Bundle TTL(s): " << config.bundleTTL / 1000000 << std::endl;
        file << "Comm Range (m): " << config.commRange << std::endl;
        file << "Ferry Height (m): " << config.ferryHeight << std::endl;
        file << "Ferry Speed (m/s)): " << config.ferrySpeed << std::endl;
        file << "Ground Buffer Size: " << config.groundBufferSize << std::endl;
        file << "Ferry Buffer Size: " << config.ferryBufferSize << std::endl;
        file << "Number of Grounds: " << config.nGrounds << std::endl;
        file << "Number of Ferries: " << config.nFerrys << std::endl;
        file << "Enable Ferry Communication: " << config.enableFerryComm << std::endl;
        // file << "Waypoint Selection Mode: " << config.waypointSelectMode << std::endl;

        file << "--REPORT" << std::endl;
        file << "Bundle Count: " << bundleCount << std::endl;
        file << "Bundle Reached Destination: " << bundleReachedDestination << std::endl;
        file << "Total Delay (s): " << totalDelay.GetSeconds() << std::endl;
        file << "Average Delay (s): " << totalDelay.GetSeconds() / bundleCount << std::endl;
        file << "Average Top 5% Delay (s): " << averageTop5pDelay << std::endl;
        file << "Average Top 10% Delay (s): " << averageTop10pDelay << std::endl;
        file << "Average Top 20% Delay (s): " << averageTop20pDelay << std::endl;
        file << "Average Top 30% Delay (s): " << averageTop30pDelay << std::endl;
        file << "Average Top 50% Delay (s): " << averageTop50pDelay << std::endl;
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
        file << "Low Connection (30%): " << lowConnection_30p << std::endl;
        file << "Low Connection (50%): " << lowConnection_50p << std::endl;
        file.close();
    }
}

#endif // REPORT_H