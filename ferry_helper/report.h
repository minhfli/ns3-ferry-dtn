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


    void Export() {
        std::ofstream file(reportFileName);
        file << "Bundle Count: " << bundleCount << std::endl;
        file << "Bundle Reached Destination: " << bundleReachedDestination << std::endl;
        file << "Total Delay: " << totalDelay.GetSeconds() << " seconds" << std::endl;
        file << "Average Delay: " << totalDelay.GetSeconds() / bundleCount << " seconds" << std::endl;
        file << "Average Hops: " << (double)totalHopReachedDestination / bundleReachedDestination << std::endl;
        file << "Delivery Ratio: " << (double)bundleReachedDestination / bundleCount << std::endl;
        file << "Foward Ratio: " << (double)bundleFowardCount / bundleCount << std::endl;
        file.close();
    }


}

#endif // REPORT_H