#ifndef MTSP_HELPER_H
#define MTSP_HELPER_H

#include "tsp-helper.h"
#include "cluster-helper.h"
#include "config.h"

#include <iostream>
#include <vector>
#include <numeric>
#include <random>
#include <algorithm>
#include <set>

double Compute_mTSP_cap_dl_vt_cost(
    const std::vector<uint32_t>& order, const std::vector<point2D>& points,
    const uint32_t k,
    const point2D& hubPos,
    const double startTime,
    const double routeLengthCap,
    const std::vector<std::vector<double>>& deadlines, // deadline must be sorted
    const std::vector<double>& timeFromLastVisit) {

    double currentRouteLength = 0;
    point2D currentPos = hubPos;
    uint32_t currentCluster = 0;

    double deadlineReward = 0;
    double timeReward = 0;
    double totalDeadline = 0;
    for (auto& dls : deadlines)
        totalDeadline += dls.size();

    for (auto idx : order) {
        if (2 * dist(points[idx], hubPos) + config.ferrySpeed * config.hoverTime > routeLengthCap) {
            continue;
        }
        if (currentRouteLength + dist(currentPos, points[idx]) + dist(points[idx], hubPos) + config.ferrySpeed * config.hoverTime > routeLengthCap) {
            currentCluster++;
            if (currentCluster >= k) {
                break;
            }
            currentRouteLength = 0;
            currentPos = hubPos;

        }

        currentRouteLength += dist(currentPos, points[idx]);

        int start = 0;
        int end = deadlines[idx].size() - 1;
        while (start <= end) {
            int mid = (start + end) / 2;
            if (deadlines[idx][mid] <= startTime + (currentRouteLength - 100) / config.ferrySpeed) {
                start = mid + 1;
            }
            else {
                end = mid - 1;
            }
        }
        deadlineReward += end + 1;

        timeReward += timeFromLastVisit[idx];

        currentRouteLength += config.ferrySpeed * config.hoverTime;
        currentPos = points[idx];
    }

    double reward = deadlineReward * config.nGrounds / (totalDeadline + 1) + timeReward / startTime;
    return -reward;
}

ClusterSolution mTSP_cap_dl_vt(
    const std::vector<point2D>& points,
    const uint32_t k,
    const point2D& hubPos,
    const double startTime,
     double routeLengthCap,
    const std::vector<std::vector<double>>& deadlines,
    const std::vector<double>& timeFromLastVisit,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000) {
    routeLengthCap -= config.hoverTime * config.ferrySpeed; // không tính thời gian chờ ở hub
    NS_LOG_UNCOND("mTSP cap + deadline + visit time");
    uint32_t POP_SIZE = population_size; // population size
    uint32_t MAX_GEN = max_generation; // max number of generation
    const uint32_t GEN_LOG_ITER = 1000;
    const double MUT_PROB = 0.2;
    const uint32_t ELITE = 20;

    uint32_t n = points.size();
    if (n == 0) { // avoid crash
        return {};
    }
    std::mt19937 gen(1337); //! MAGIC NUMBER - Fixed seed

    std::vector<TSPSolution> population(POP_SIZE);
    for (TSPSolution& sol : population) {
        sol.order = DataStructureHelper::GetIndexVector(n);
        std::shuffle(sol.order.begin(), sol.order.end(), gen);
        sol.cost = Compute_mTSP_cap_dl_vt_cost(
            sol.order, points, k, hubPos, startTime, routeLengthCap, deadlines, timeFromLastVisit
        );
    }
    std::sort(population.begin(), population.end());

    for (uint32_t generation = 0; generation <= MAX_GEN; ++generation) {
        std::vector<TSPSolution> new_population;
        for (uint32_t i = 0; i < ELITE; ++i) {
            new_population.push_back(population[i]);
        }
        while (new_population.size() < POP_SIZE) {
            int id1 = rand() % POP_SIZE;
            int id2 = rand() % POP_SIZE;
            TSPSolution sol;
            sol.order = OrderCrossover(population[id1].order, population[id2].order, gen);
            SwapMutation(sol.order, gen, MUT_PROB);
            sol.cost = Compute_mTSP_cap_dl_vt_cost(
                sol.order, points, k, hubPos, startTime, routeLengthCap, deadlines, timeFromLastVisit
            );
            new_population.push_back(sol);
        }
        std::swap(population, new_population); // faster than assignment
        std::sort(population.begin(), population.end());
        if (generation % GEN_LOG_ITER == 0) {
            NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
        }
    }
    auto order = std::min_element(population.begin(), population.end())->order;
    ClusterSolution clusters = std::vector<std::vector<uint32_t>>(k);
    double currentRouteLength = 0;
    point2D currentPos = hubPos;
    uint32_t currentCluster = 0;

    for (auto idx : order) {
        if (currentRouteLength + dist(currentPos, points[idx]) + dist(points[idx], hubPos) + config.ferrySpeed * config.hoverTime > routeLengthCap) {
            currentCluster++;
            if (currentCluster >= k) {
                break;
            }
            currentRouteLength = 0;
            currentPos = hubPos;
        }
        clusters[currentCluster].push_back(idx);
        currentRouteLength += dist(currentPos, points[idx]) + config.ferrySpeed * config.hoverTime;
        currentPos = points[idx];
    }

    return clusters;
}

#endif