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

const std::string ENABLED = "ENABLED";
const std::string DISABLED = "DISABLED";

const std::string DETERMINISTIC = "DETERMINISTIC";
const std::string PROBABILISTIC = "PROBABILISTIC";

const std::string PURE_RANDOM = "PURE_RANDOM";
const std::string RANDOM_RANGE = "RANDOM_RANGE";
const std::string PARETO_9010 = "PARETO_9010";
const std::string PARETO_8020 = "PARETO_8020";
const std::string PARETO_7030 = "PARETO_7030";
const std::string PARETO_6040 = "PARETO_6040";
const std::set<std::string> PARETO_VALUES = { PARETO_9010, PARETO_8020, PARETO_7030, PARETO_6040 };

const std::string RPDLAS_PRUNE_HALF = "RPDLAS_PRUNE_HALF";
const std::string RPDLAS_PRUNE_ONE_THIRD = "RPDLAS_PRUNE_ONE_THIRD";
const std::string RPDLAS_PRUNE_MAXIMAL = "RPDLAS_PRUNE_MAXIMAL";
const std::string RPDLAS_PRUNE_CLUSTER = "RPDLAS_PRUNE_CLUSTER";
const std::string RPDLAS_PRUNE_ONE = "RPDLAS_PRUNE_ONE";

const std::string RPDLAS_NO_REROUTE_COLLECT_INROUTE = "RPDLAS_NO_REROUTE_COLLECT_INROUTE";
const std::string RPDLAS_REROUTE_INSERT = "RPDLAS_REROUTE_INSERT";
const std::string RPDLAS_REROUTE_OPTIMIZED = "RPDLAS_REROUTE_OPTIMIZED";

const std::string MRDLAS_ONE_ROUTE_EACH = "MRDLAS_ONE_ROUTE_EACH";
const std::string MRDLAS_ONE_ROUTE_2_FERRY = "MRDLAS_ONE_ROUTE_2_FERRY";

const std::string DRC_ONE_CENTER = "DRC_ONE_CENTER";
const std::string DRC_TWO_CENTER = "DRC_TWO_CENTER";
const std::string DRC_GABRIEL = "DRC_GABRIEL";

const std::string REP_DTN_EPIDEMIC = "EPIDEMIC";
const std::string REP_DTN_SNW = "SPRAY_AND_WAIT";

struct Config {
    int randSeed = 1337;
    std::string ALGORITHM_NAME = "SIRA";
    std::string SIMULATION_RUN = "1";
    std::string REPORT_BATCH = "default";
    bool enableVisualization = true;
    bool skipIfExist = false;
    // sim config
    double simTime = 5000.0; // seconds
    double warmupTime = 300.0; // seconds
    double commRange = 150.0; // meters
    double udcRadius = 120.0; // meters, use as safe radius
    double ferryHeight = 50.0; // meters
    double areaWidth = 4000; // meters
    double areaPadding = 0; // padding to avoid node on edge of the map
    uint32_t nGrounds = 25;
    uint32_t nFerrys = 5;

    // UDC based config
    bool UDC_enabled = false;
    double UDC_radius = 120;
    double UDC_genMinDist = 10;

    // node config
    double beaconInterval = 4.0;    // seconds
    double beaconRandomnes = 1.0; // seconds
    uint64_t contactTimeout = 3000000; // 3 seconds, time between last contact of 2 node that it accept new beacon and start new protocol session
    uint32_t jitterAmount = 500; // 0.5 - 1.5 milisecconds

    // mobility
    double ferrySpeed = 12.0;      // m/s
    double hoverTime = 10.0; // sec, everytime an UAV reach a waypoint, it will hover for this time 
    /**
    * if ferry node is scheduled to hover (stay at the same place), how long does it wait to reschedule it mobility ?
    * this is to avoid too frequently scheduling, with especially with pigeon dtn app
    * */
    double mobilityWaitTime = 5.0; // (seconds) 

    // bundles
    uint32_t groundBufferSize = 100; // bundles, ground node will only hold bundle that it created
    uint32_t ferryBufferSize = 20;  // bundles

    std::string bundleGenSourceScheduler = RANDOM_RANGE;
    std::string bundleGenDestinationSheduler = PARETO_7030;
    bool bundleGenParetoMatch = false;

    // double bundleGenRate = 5.0; // 1 bundle every ... seconds
    double maxBaseBundleGenRate = 30.0;
    double minBaseBundleGenRate = 30.0;
    // std::string
    uint32_t bundleTTL = 300000000; // 300 seconds (microsec) ~ 5 min
    uint32_t bundleAckTimeout = 500000; // 0.5 seconds (microsec)
    double minExpectedArrivalDifference = 5; // (seconds), min expected arrival difference (Beacon interval) for 2 node to exchange bundle information and accept bundle transfer

    bool enableFerryComm = false;

    // physical payload size of a chunk and bundle
    uint32_t chunkPayload = 1024; // 1KB, currently, we will only use this
    uint32_t bundlePayload = 102400; // 100KB, chunking will be implemented later
    uint32_t maxBundlePerSumary = 125; // 125 bundle per sumary vector -> 8 * 125 + 1 + 4 = 1005 byte

    // visualization config
    uint32_t positionLogInterval = 1000; // ms ~ 0.25s

    // DTN config
    std::string replicationBaseDtnAppMode = REP_DTN_SNW;
    uint32_t epidemic_maxHop = 10;
    uint32_t SnW_replications = 16;
    bool SnW_binary = true;

    // algorithm config
    uint32_t PIGEON_return_mode = PIGEON_RETURN_CONTINUE;

    bool SR_PIGEON_V2_addlvt = false; // add time since last visit value to node score calculation
    bool SR_PIGEON_V2_vtModePredict = true; //   

    bool TABAF_weightedDeadline = false;
    bool TABAF_shareVisitTime = true;

    bool TABADLA_addBundleValue = false;
    uint32_t TABADLA_topK = 10;

    std::string RPDLAS_operationMode = RPDLAS_NO_REROUTE_COLLECT_INROUTE;
    std::string RPDLAS_pruneMode = RPDLAS_PRUNE_HALF;

    std::string MRDLAS_routeMode = MRDLAS_ONE_ROUTE_EACH;
    bool MRDLAS_weightedDeadline = false;

    // waypoint selection config, for Tabaf and its derived algorithm
    std::string waypointSelectMode = DETERMINISTIC;

    std::string DRC_graphMode = DRC_ONE_CENTER;
    uint32_t DRC_refineIterations = 1000;
    uint32_t DRC_sampleCount = 1000;
    double DRC_lastContactTimeout = 10;
    double DRC_maxWaitTime = 900; // 900 sec

    uint32_t HUB_nHubs = 1;

    bool CHUB_virtualHub = true;
    bool CHUB_routeExtend = false;
    bool CHUB_reWait = false;
    bool CHUB_squareREReward = false;



} config;

Time GetJitter() {
    return MicroSeconds(m_rand->GetInteger(config.jitterAmount, config.jitterAmount * 3));
}

struct AlgorithmSpecificSettings {
    bool sendRouteInHello = true; // always send route Header in Hello message if not disabled by specific algorithm
    bool sendVisitTimeInHello = true;
    bool sendBundleCountInHello = true; // this is must be true for all base-dtn-app derived app
} algoConfig;


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
    cmd.AddValue("warmupTime", "Warmup time", config.warmupTime);
    cmd.AddValue("commRange", "Communication range", config.commRange);
    cmd.AddValue("udcRadius", "Unit disk radius", config.udcRadius);
    cmd.AddValue("ferryHeight", "Ferry height", config.ferryHeight);
    cmd.AddValue("areaWidth", "Area width", config.areaWidth);
    cmd.AddValue("nGrounds", "Number of ground nodes", config.nGrounds);
    cmd.AddValue("nFerrys", "Number of ferry nodes", config.nFerrys);
    // cmd.AddValue("beaconInterval", "Beacon interval", config.beaconInterval);
    // cmd.AddValue("beaconRandomnes", "Beacon randomness", config.beaconRandomnes);
    // cmd.AddValue("jitterAmount", "Jitter amount", config.jitterAmount);
    cmd.AddValue("ferrySpeed", "Ferry speed", config.ferrySpeed);
    cmd.AddValue("hoverTime", "Hover time", config.hoverTime);
    cmd.AddValue("groundBufferSize", "Ground buffer size", config.groundBufferSize);
    cmd.AddValue("ferryBufferSize", "Ferry buffer size", config.ferryBufferSize);
    // cmd.AddValue("bundleGenRate", "Bundle generation rate", config.bundleGenRate);
    cmd.AddValue("minGenRate", "Min bundle generation rate", config.minBaseBundleGenRate);
    cmd.AddValue("maxGenRate", "Max bundle generation rate", config.maxBaseBundleGenRate);
    cmd.AddValue("genSrcScheduler", "Bundle generation source scheduler", config.bundleGenSourceScheduler);
    cmd.AddValue("genDstScheduler", "Bundle generation destination scheduler", config.bundleGenDestinationSheduler);
    cmd.AddValue("genParetoMatch", "Bundle generation pareto match", config.bundleGenParetoMatch);

    cmd.AddValue("bundleTTL", "Bundle TTL", config.bundleTTL);
    cmd.AddValue("ferryComm", "Enable ferry communication", config.enableFerryComm);
    // cmd.AddValue("bundleAckTimeout", "Bundle ACK timeout", config.bundleAckTimeout);
    cmd.AddValue("SnW_replications", "SnW replications", config.SnW_replications);

    // ----- algorithm specific config -----
    cmd.AddValue("pigeonReturn", "Pigeon return mode", config.PIGEON_return_mode);
    cmd.AddValue("TABAF_shareVisitTime", "TABAF share visit time", config.TABAF_shareVisitTime);
    cmd.AddValue("TABADLA_addBundleValue", "TABADLA, add bundle value", config.TABADLA_addBundleValue);
    cmd.AddValue("waypointSelectMode", "Waypoint selection mode", config.waypointSelectMode);
    cmd.AddValue("SR_PIGEON_V2_addlvt", "SR_PIGEON_V2, add lvt", config.SR_PIGEON_V2_addlvt);
    cmd.AddValue("RPDLAS_operationMode", "RPDLAS operation mode", config.RPDLAS_operationMode);
    cmd.AddValue("RPDLAS_pruneMode", "RPDLAS prune mode", config.RPDLAS_pruneMode);
    cmd.AddValue("MRDLAS_routeMode", "MRDLAS route mode", config.MRDLAS_routeMode);
    cmd.AddValue("MRDLAS_weightedDeadline", "MRDLAS weighted deadline", config.MRDLAS_weightedDeadline);
    cmd.AddValue("TABAF_weightedDeadline", "TABAF weighted deadline", config.TABAF_weightedDeadline);
    cmd.AddValue("DRC_graphMode", "DRC graph mode", config.DRC_graphMode);
    cmd.AddValue("HUB_nHubs", "Number of hubs", config.HUB_nHubs);
    cmd.AddValue("CHUB_virtualHub", "Use virtual hub", config.CHUB_virtualHub);
    cmd.AddValue("CHUB_routeExtend", "Use route extension", config.CHUB_routeExtend);
    cmd.AddValue("CHUB_reWait", "Use re-wait", config.CHUB_reWait);
    cmd.AddValue("CHUB_squareREReward", "Use squared route extend reward", config.CHUB_squareREReward);
    cmd.Parse(argc, argv);
    return;
}

#endif // CONFIG