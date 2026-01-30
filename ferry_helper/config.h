#ifndef CONFIG
#define CONFIG

#include "ns3/core-module.h"    

struct Config {
    // sim config
    double simTime = 3000.0; // seconds
    double commRange = 150.0; // meters
    double areaWidth = 1000; // meters
    uint32_t nGrounds = 5;
    uint32_t nFerrys = 10;

    bool enable_background = false;

    // node config
    double beaconInterval = 5.0;    // seconds
    double beaconRandomness = 3.0; // seconds
    uint32_t jitterAmount = 1000; // 1 milisecconds, used for exchange message

    double ferrySpeed = 15.0;      // m/s
    uint16_t dtnPort = 9000;
    uint32_t groundBufferSize = 5; // bundles, ground node will only hold bundle that it created
    uint32_t ferryBufferSize = 20;  // bundles

    uint32_t bundleTTL = 100000000; // 100 seconds (microsec)
    uint32_t bundleAckTimeout = 500000; // 0.5 seconds (microsec)

    // physical payload size of a chunk and bundle
    uint32_t chunkPayload = 1024; // 1KB, currently, we will only use this
    uint32_t bundlePayload = 102400; // 100KB, chunking will be implemented later

} config;

struct Logging {
    uint32_t bundleCount = 0;
    uint32_t bundleSuccessForward = 0;
    uint32_t bundleDropped = 0;
} simlog;

#endif // CONFIG