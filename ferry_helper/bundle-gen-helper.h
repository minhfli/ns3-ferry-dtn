#ifndef BUNDLE_GEN_HELPER
#define BUNDLE_GEN_HELPER

#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <set>

#include "global.h"
#include "config.h"

using namespace ns3;

namespace BundleGenerationHelper {

    std::set<uint32_t> srcVIN; // VIN: Very Important Node
    std::set<uint32_t> dstVIN; // VIN: Very Important Node
    double totalReceiveChance = 0;

    std::set<uint32_t> SampleTopK(uint32_t k, uint32_t n) {
        if (k > n) k = n;
        std::vector<uint32_t> a(n);
        for (uint32_t i = 0; i < n; i++)
            a[i] = i;

        std::set<uint32_t> result;
        // Fisher-Yates Shuffle 
        for (uint32_t i = 0; i < k; i++) {
            uint32_t j = m_rand->GetInteger(i, n - 1);
            std::swap(a[i], a[j]);
            result.insert(a[i]);
        }
        return result;
    }

    void InitSourceScheduler() {
        groundGenRate.resize(config.nGrounds);
        if (config.bundleGenSourceScheduler == RANDOM_RANGE) {
            for (uint32_t i = 0; i < config.nGrounds; i++) {
                groundGenRate[i] = m_rand->GetValue(config.minBaseBundleGenRate, config.maxBaseBundleGenRate);
            }
            return;
        }
        NS_LOG_UNCOND("ERROR: Source scheduler not implemented: " << config.bundleGenSourceScheduler);
        NS_ASSERT(false);
    }


    void InitDestinationScheduler() {
        groundReceiveChances.resize(config.nGrounds);
        if (config.bundleGenDestinationSheduler == PURE_RANDOM) {
            for (uint32_t i = 0; i < config.nGrounds; i++) {
                groundReceiveChances[i] = 1;
            }
            totalReceiveChance = config.nGrounds;
            return;
        }
        double pareto = 0;
        if (config.bundleGenDestinationSheduler == PARETO_9010) pareto = 90;
        if (config.bundleGenDestinationSheduler == PARETO_8020) pareto = 80;
        if (config.bundleGenDestinationSheduler == PARETO_7030) pareto = 70;
        if (config.bundleGenDestinationSheduler == PARETO_6040) pareto = 60;
        if (pareto != 0) {
            // PARETO schedule: p% messages go to (100-p)% nodes
            uint32_t k = config.nGrounds * pareto / 100;
            dstVIN = SampleTopK(k, config.nGrounds);
            double VIN_receiveChance = (1.0 * pareto * pareto) / (100.0 - pareto) / (100.0 - pareto);

            for (uint32_t i = 0; i < config.nGrounds; i++) {
                if (dstVIN.find(i) != dstVIN.end()) {
                    groundReceiveChances[i] = VIN_receiveChance;
                }
                else {
                    groundReceiveChances[i] = 1;
                }
                totalReceiveChance += groundReceiveChances[i];
            }
            return;
        }

        NS_LOG_UNCOND("ERROR: Destination scheduler not implemented: " << config.bundleGenDestinationSheduler);
        NS_ASSERT(false);
    }

    void Init() {
        InitSourceScheduler();
        InitDestinationScheduler();
    }

    uint32_t ChoseBundleDestination(uint32_t srcIp) {
        double chance = bundleGenRand->GetValue(0.0, totalReceiveChance - nodeReceiveChance(srcIp));
        uint32_t lastnode = 0;
        for (uint32_t i = 0; i < config.nGrounds; i++) {
            if (groundNodeIps[i].Get() == srcIp) continue;
            chance -= groundReceiveChances[i];
            lastnode = i;
            if (chance <= 0) {
                return groundNodeIps[i].Get();
            }
        }
        return groundNodeIps[lastnode].Get();
    }

};

#endif // BUNDLE_GEN_HELPER