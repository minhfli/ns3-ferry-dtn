#ifndef CONFIG
#define CONFIG

#include <string>
#include "ns3/core-module.h"    
#include "ns3/network-module.h"

using namespace ns3;

struct Config {
    std::string SIMULATION_NAME = "DTN";
    std::string SIMULATION_RUN = "1";
    // sim config
    double simTime = 5000.0; // seconds
    double commRange = 150.0; // meters
    double ferryHeight = 50.0; // meters
    double areaWidth = 4000; // meters
    uint32_t nGrounds = 25;
    uint32_t nFerrys = 5;

    bool enable_background = false;

    // node config
    double beaconInterval = 3.0;    // seconds
    double beaconRandomness = 1.0; // seconds
    uint32_t jitterAmount = 1000; // 1 milisecconds, used for exchange message

    double ferrySpeed = 15.0;      // m/s
    uint16_t dtnPort = 9000;
    uint32_t groundBufferSize = 1000000000; // bundles, ground node will only hold bundle that it created
    uint32_t ferryBufferSize = 20;  // bundles

    double bundleGenRate = 5.0; // 1 bundle every ... seconds
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

void ParseConfig(int argc, char* argv[]) {
    CommandLine cmd;
    // commented values are fixed variables
    cmd.AddValue("name", "Simulation Name", config.SIMULATION_NAME);
    cmd.AddValue("run", "Simulation Run", config.SIMULATION_RUN);
    cmd.AddValue("simTime", "Simulation time", config.simTime);
    cmd.AddValue("commRange", "Communication range", config.commRange);
    cmd.AddValue("ferryHeight", "Ferry height", config.ferryHeight);
    cmd.AddValue("areaWidth", "Area width", config.areaWidth);
    cmd.AddValue("nGrounds", "Number of ground nodes", config.nGrounds);
    cmd.AddValue("nFerrys", "Number of ferry nodes", config.nFerrys);
    // cmd.AddValue("beaconInterval", "Beacon interval", config.beaconInterval);
    // cmd.AddValue("beaconRandomness", "Beacon randomness", config.beaconRandomness);
    // cmd.AddValue("jitterAmount", "Jitter amount", config.jitterAmount);
    cmd.AddValue("ferrySpeed", "Ferry speed", config.ferrySpeed);
    // cmd.AddValue("dtnPort", "Dtn port", config.dtnPort);
    cmd.AddValue("groundBufferSize", "Ground buffer size", config.groundBufferSize);
    cmd.AddValue("ferryBufferSize", "Ferry buffer size", config.ferryBufferSize);
    cmd.AddValue("bundleGenRate", "Bundle generation rate", config.bundleGenRate);
    cmd.AddValue("bundleTTL", "Bundle TTL", config.bundleTTL);
    // cmd.AddValue("bundleAckTimeout", "Bundle ACK timeout", config.bundleAckTimeout);

    cmd.Parse(argc, argv);
    return;
}

#endif // CONFIG