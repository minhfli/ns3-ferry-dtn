#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "datatypes.h"
#include <vector>
#include <random>
#include <unordered_map>

using namespace ns3;

constexpr uint8_t NODE_TYPE_GROUND = 0;
constexpr uint8_t NODE_TYPE_FERRY = 1;
constexpr uint8_t MODE_DEFAULT = 0;
constexpr uint8_t MODE_FERRY = 1;
constexpr uint8_t MODE_PIGEON = 2;

// ===========================================================================
// Global variables
// ===========================================================================

// global RNG
std::mt19937 globalRNG(1337); //! MAGIC NUMBER - Fixed seed

Ptr<UniformRandomVariable> m_rand;
// seperate rng for bundle generation, so the bundle generation will be the same across simulations
Ptr<UniformRandomVariable> bundleGenRand;
//? ASSUMPTION: all node have these infomation

// array of node ip !!! THIS IS NOT A MAP
std::vector<Ipv4Address> groundNodeIps;
// array of node position !!! THIS IS NOT A MAP
std::vector<point2D> groundNodePos;
// array of receive chances of ground node
std::vector<double> groundReceiveChances;
// array of bundle generation rate of ground node (sec/bundle)
std::vector<double> groundGenRate;
// array of node ip !!! THIS IS NOT A MAP
std::vector<Ipv4Address> ferryIps;
// array of unit disk centers (pos, coverNodes)!!! THIS IS NOT A MAP 
std::vector<waypoint2D> UdcWaypoints;


// mapping from IP to type
std::unordered_map<uint32_t, uint8_t> nodeType;
// mapping from IP to group
std::unordered_map<uint32_t, uint8_t> nodeGroup;
// mapping from IP to node Id (g0, g1, ...)
std::unordered_map<uint32_t, std::string> nodeId;
// mapping from IP bundle generation rate, (bundle/sec) only tabara algorithm use this
std::unordered_map<uint32_t, double> nodeGenRate;
// mapping from IP to node index, note that both ground and ferry start at 0
std::unordered_map<uint32_t, uint32_t> nodeIndex;


// get node id without prefix
uint32_t rawNodeId(uint32_t ip) {
    return std::stoi(nodeId[ip].substr(1));
}

point2D nodePos(uint32_t ip) {
    return groundNodePos[rawNodeId(ip)];
}

double nodeReceiveChance(uint32_t ip) {
    return groundReceiveChances[rawNodeId(ip)];
}


#endif