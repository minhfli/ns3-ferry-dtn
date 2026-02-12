#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "datatypes.h"
#include <vector>
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

Ptr<UniformRandomVariable> m_rand;
//? ASSUMPTION: all node have these infomation
std::vector<Ipv4Address> groundNodeIps;
std::vector<point2D> groundNodePos;
// mapping from IP to type
std::unordered_map<uint32_t, uint8_t> nodeType;
// mapping from IP to group
std::unordered_map<uint32_t, uint8_t> nodeGroup;
// mapping from IP to node Id (g0, g1, ...)
std::unordered_map<uint32_t, std::string> nodeId;

// get node id without prefix
uint32_t rawNodeId(uint32_t ip) {
    return std::stoi(nodeId[ip].substr(1));
}

point2D nodePos(uint32_t ip) {
    return groundNodePos[rawNodeId(ip)];
}

#endif