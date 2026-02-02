#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <vector>
#include <unordered_map>

using namespace ns3;

constexpr uint8_t NODE_TYPE_GROUND = 0;
constexpr uint8_t NODE_TYPE_FERRY = 1;

// ===========================================================================
// Global variables
// ===========================================================================

Ptr<UniformRandomVariable> m_rand;
std::vector<Ipv4Address> groundNodeIps;
// mapping from IP to type, all node have this infomation
std::unordered_map<uint32_t, uint8_t> nodeType;
// mapping from IP to group, all node have this infomation
std::unordered_map<uint32_t, uint8_t> nodeGroup;
// mapping from IP to node Id (g0, g1, ...)
std::unordered_map<uint32_t, std::string> nodeId;

#endif