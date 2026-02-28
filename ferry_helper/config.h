#ifndef CONFIG
#define CONFIG

#include <string>
#include "ns3/core-module.h"    
#include "ns3/network-module.h"

using namespace ns3;

constexpr uint8_t SELECT_MODE_RANDOM_MINIMUM = 0;
constexpr uint8_t SELECT_MODE_RANDOM_MAXIMUM = 1;
constexpr uint8_t SELECT_MODE_PROBALISTC = 2;

constexpr uint8_t PIGEON_RETURN_CONTINUE = 0;
constexpr uint8_t PIGEON_RETURN_CLOSET = 1;

struct Config {
    int randSeed = 1337;
    std::string ALGORITHM_NAME = "SIRA";
    std::string SIMULATION_RUN = "1";
    std::string REPORT_BATCH = "default";
    bool enableVisualization = true;
    bool skipIfExist = false;
    // sim config
    double simTime = 5000.0; // seconds
    double startGeneraionTime = 300.0; // seconds, warmup before
    double commRange = 150.0; // meters
    double ferryHeight = 50.0; // meters
    double areaWidth = 4000; // meters
    double areaPadding = 100; // padding to avoid node on edge of the map
    uint32_t nGrounds = 25;
    uint32_t nFerrys = 5;

    // node config
    double beaconInterval = 5.0;    // seconds
    double beaconRandomnes = 1.0; // seconds
    uint64_t contactTimeout = 3000000; // 3 seconds, time between last contact of 2 node that it accept new beacon and start new protocol session
    uint32_t jitterAmount = 500; // 0.5 - 1.5 milisecconds

    // mobility
    double ferrySpeed = 15.0;      // m/s
    /**
    * if ferry node is scheduled to hover (stay at the same place), how long does it wait to reschedule it mobility ?
    * this is to avoid too frequently scheduling, with especially with pigeon dtn app
    * */
    double mobilityWaitTime = 5.0; // (seconds) 

    // bundles
    uint32_t groundBufferSize = 100; // bundles, ground node will only hold bundle that it created
    uint32_t ferryBufferSize = 20;  // bundles

    double bundleGenRate = 5.0; // 1 bundle every ... seconds
    uint32_t bundleTTL = 300000000; // 300 seconds (microsec) ~ 5 min
    uint32_t bundleAckTimeout = 500000; // 0.5 seconds (microsec)
    double minExpectedArrivalDifference = 10; // (seconds), min expected arrival difference (2 * Beacon interval) for 2 node to exchange bundle information and accept bundle transfer

    bool enableFerryComm = false;

    // physical payload size of a chunk and bundle
    uint32_t chunkPayload = 1024; // 1KB, currently, we will only use this
    uint32_t bundlePayload = 102400; // 100KB, chunking will be implemented later

    // visualization config
    uint32_t positionLogInterval = 250; // ms ~ 0.25s

    uint32_t PIGEON_return_mode = PIGEON_RETURN_CONTINUE;

    // waypoint selection config, for Tabaf and its derived algorithm
    uint32_t TABAF_waypointSelectMode = SELECT_MODE_RANDOM_MAXIMUM;

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
    cmd.AddValue("seed", "Random seed", config.randSeed);
    cmd.AddValue("name", "Simulation Name", config.ALGORITHM_NAME);
    cmd.AddValue("run", "Simulation Run", config.SIMULATION_RUN);
    cmd.AddValue("batch", "Report batch name", config.REPORT_BATCH);
    cmd.AddValue("vi", "Enable visualization", config.enableVisualization);
    cmd.AddValue("skip", "Skip if exist", config.skipIfExist);
    // -- general config --
    cmd.AddValue("simTime", "Simulation time", config.simTime);
    cmd.AddValue("commRange", "Communication range", config.commRange);
    cmd.AddValue("ferryHeight", "Ferry height", config.ferryHeight);
    cmd.AddValue("areaWidth", "Area width", config.areaWidth);
    cmd.AddValue("nGrounds", "Number of ground nodes", config.nGrounds);
    cmd.AddValue("nFerrys", "Number of ferry nodes", config.nFerrys);
    // cmd.AddValue("beaconInterval", "Beacon interval", config.beaconInterval);
    // cmd.AddValue("beaconRandomnes", "Beacon randomness", config.beaconRandomnes);
    // cmd.AddValue("jitterAmount", "Jitter amount", config.jitterAmount);
    cmd.AddValue("ferrySpeed", "Ferry speed", config.ferrySpeed);
    cmd.AddValue("groundBufferSize", "Ground buffer size", config.groundBufferSize);
    cmd.AddValue("ferryBufferSize", "Ferry buffer size", config.ferryBufferSize);
    cmd.AddValue("bundleGenRate", "Bundle generation rate", config.bundleGenRate);
    cmd.AddValue("bundleTTL", "Bundle TTL", config.bundleTTL);
    cmd.AddValue("ferryComm", "Enable ferry communication", config.enableFerryComm);
    // cmd.AddValue("bundleAckTimeout", "Bundle ACK timeout", config.bundleAckTimeout);

    // ----- algorithm specific config -----
    cmd.AddValue("pigeonReturn", "Pigeon return mode", config.PIGEON_return_mode);
    cmd.AddValue("tabafSelect", "Waypoint selection mode", config.TABAF_waypointSelectMode);

    cmd.Parse(argc, argv);
    return;
}

#endif // CONFIG