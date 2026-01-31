#ifndef CONFIG
#define CONFIG

#include "ns3/core-module.h"    

using namespace ns3;

struct Config {
    // sim config
    double simTime = 1000.0; // seconds
    double commRange = 150.0; // meters
    double ferryHeight = 50.0; // meters
    double areaWidth = 1000; // meters
    uint32_t nGrounds = 5;
    uint32_t nFerrys = 1;

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

    // visualization config
    uint32_t positionLogInterval = 250; // ms ~ 0.25s

} config;

struct Logging {
    uint32_t bundleCount = 0;
    uint32_t bundleSuccessForward = 0;
    uint32_t bundleDropped = 0;
} simlog;

struct SimulationVariablePointer {
    uint32_t nGrounds = 0;
    uint32_t nFerrys = 0;
    NodeContainer* groundNodes;
    NodeContainer* ferryNodes;
} simVar;

#endif // CONFIG