#ifndef CONFIG
#define CONFIG

#include "ns3/core-module.h"    
#include "ns3/network-module.h"

using namespace ns3;

struct Config {
    // sim config
    double simTime = 5000.0; // seconds
    double commRange = 120.0; // meters
    double ferryHeight = 50.0; // meters
    double areaWidth = 2000; // meters
    uint32_t nGrounds = 10;
    uint32_t nFerrys = 3;

    bool enable_background = false;

    // node config
    double beaconInterval = 3.0;    // seconds
    double beaconRandomness = 1.0; // seconds
    uint32_t jitterAmount = 1000; // 1 milisecconds, used for exchange message

    double ferrySpeed = 15.0;      // m/s
    uint16_t dtnPort = 9000;
    uint32_t groundBufferSize = 5; // bundles, ground node will only hold bundle that it created
    uint32_t ferryBufferSize = 20;  // bundles

    uint32_t bundleTTL = 300000000; // 300 seconds (microsec) ~ 5 min
    uint32_t bundleAckTimeout = 500000; // 0.5 seconds (microsec)

    // physical payload size of a chunk and bundle
    uint32_t chunkPayload = 1024; // 1KB, currently, we will only use this
    uint32_t bundlePayload = 102400; // 100KB, chunking will be implemented later

    // visualization config
    uint32_t positionLogInterval = 250; // ms ~ 0.25s

} config;


struct SimulationVariablePointer {
    uint32_t nGrounds = 0;
    uint32_t nFerrys = 0;
    NodeContainer* groundNodes;
    NodeContainer* ferryNodes;
} simVar;

#endif // CONFIG