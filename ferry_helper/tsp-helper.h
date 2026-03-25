#ifndef TSP_HELPER_H
#define TSP_HELPER_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <limits>
#include <set>

#include "datatypes.h"
#include "global.h"


struct TSPSolution {
    std::vector<uint32_t> order;
    double cost;
    bool operator<(const TSPSolution& other) const {
        return cost < other.cost;
    }

    void rollTo1() { //! only work if index 1 is in the order, and route size > 2
        uint32_t oneIndex = 0;
        for (uint32_t i = 0; i < order.size(); i++) {
            if (order[i] == 1) {
                oneIndex = i;
                break;
            }
        }
        std::rotate(order.begin(), order.begin() + oneIndex, order.end());
        if (order[1] > order[order.size() - 1]) {
            std::reverse(order.begin() + 1, order.end());
        }
    }

    bool checkEqual(const TSPSolution& other) const {
        if (std::abs(cost - other.cost) > 1e-6) {
            return false;
        }
        for (uint32_t i = 0; i < order.size(); i++) {
            if (order[i] != other.order[i]) {
                return false;
            }
        }
        return true;
    }
};

#pragma region "CLASSIC TSP"
/* ================================
   TSP COST
================================ */
inline double ComputeCost(const std::vector<uint32_t>& order,
                          const std::vector<point2D>& points) {
    double cost = 0.0;
    for (size_t i = 0; i < order.size() - 1; ++i) {
        cost += dist(points[order[i]], points[order[i + 1]]);
    }
    cost += dist(points[order.back()], points[order.front()]);
    return cost;
}

/* ================================
   ORDER CROSSOVER (OX)
================================ */
inline std::vector<uint32_t> OrderCrossover(
        const std::vector<uint32_t>& p1,
        const std::vector<uint32_t>& p2,
        std::mt19937& gen) {

    size_t n = p1.size();
    std::uniform_int_distribution<size_t> distIdx(0, n - 1);
    size_t l = distIdx(gen);
    size_t r = distIdx(gen);
    if (l > r) std::swap(l, r);

    std::vector<uint32_t> child(n, UINT32_MAX);

    for (size_t i = l; i <= r; ++i)
        child[i] = p1[i];

    size_t idx = (r + 1) % n;
    for (size_t i = 0; i < n; ++i) {
        uint32_t city = p2[(r + 1 + i) % n];
        if (std::find(child.begin(), child.end(), city) == child.end()) {
            child[idx] = city;
            idx = (idx + 1) % n;
        }
    }
    return child;
}

/* ================================
   MUTATION
================================ */
inline void SwapMutation(std::vector<uint32_t>& order,
                         std::mt19937& gen,
                         double prob) {
    std::uniform_real_distribution<double> p(0.0, 1.0);
    if (p(gen) < prob) {
        std::uniform_int_distribution<size_t> d(0, order.size() - 1);
        size_t i = d(gen);
        size_t j = d(gen);
        std::swap(order[i], order[j]);
    }
}

/* ================================
   MAIN GA
================================ */
std::vector<uint32_t> TSPClassicGA(const std::vector<point2D>& points, const std::set<uint32_t>& excludeIdx = {}, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
    NS_LOG_UNCOND("TSP Classic GA");
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

    /* Initialize population */
    std::vector<uint32_t> included;
    for (uint32_t i = 0; i < n; i++) {
        if (excludeIdx.find(i) == excludeIdx.end()) {
            included.push_back(i);
        }
    }
    if (included.size() <= 3) {
        return included;
    }

    std::vector<TSPSolution> population(POP_SIZE);
    for (TSPSolution& sol : population) {
        sol.order = included;
        std::shuffle(sol.order.begin(), sol.order.end(), gen);
        sol.cost = ComputeCost(sol.order, points);
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
            sol.cost = ComputeCost(sol.order, points);
            new_population.push_back(sol);
        }
        std::swap(population, new_population); // faster than assignment
        std::sort(population.begin(), population.end());
        if (generation % GEN_LOG_ITER == 0) {
            NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
        }
    }

    return std::min_element(population.begin(), population.end())->order;
}

std::vector<uint32_t> TSPTwoOptOptimize(const std::vector<point2D>& points, const std::vector<uint32_t>& baseOrder) {
    if (baseOrder.size() <= 1) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    double bestCost = ComputeCost(order, points);
    bool improve = true;
    while (improve) {
        improve = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 1; j < order.size(); ++j) {
                std::reverse(order.begin() + i, order.begin() + j + 1);
                double newCost = ComputeCost(order, points);
                if (newCost < bestCost) {
                    bestCost = newCost;
                    improve = true;
                }
                else {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                }
            }
        }
    }
    return order;
}

std::vector<uint32_t> TSPHelper(const std::vector<point2D>& points, const std::set<uint32_t>& excludeIdx = {}, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
    std::vector<uint32_t> order = TSPClassicGA(points, excludeIdx, population_size, max_generation);
    order = TSPTwoOptOptimize(points, order);
    return order;
}
#pragma endregion

#pragma region "DEADLINE BASED TSP"

uint32_t ComputeDeadlineCost(const std::vector<uint32_t>& order,
    const std::vector<point2D>& points,
    const std::vector<std::vector<double>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed
) {
    uint32_t cost = 0;
    double currentTime = starting_time;
    point2D currentPos = starting_pos;
    for (size_t i = 0; i < order.size(); ++i) {
        point2D nextPos = points[order[i]];
        double distance = dist(currentPos, nextPos);
        double travelTime = distance / speed;
        currentTime += travelTime;

        // check deadlines
        for (double deadline : deadlines[order[i]]) {
            if (currentTime > deadline) {
                cost += 1; // large penalty for missing deadline
            }
        }

        currentPos = nextPos;
    }
    // NS_LOG_UNCOND("Hello?");

    return cost;
}

/**
 * Find a route that go through all the points so that the most of deadlines are sastisfied
 * @return first: route, second: is true if all deadlines are sastisfied
 *
 */
std::vector<uint32_t> TSPDeadlineBasedGA(
    const std::vector<point2D>& points,
    const std::vector<std::vector<double>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000,
    uint32_t* best_cost = nullptr
) {
    uint32_t POP_SIZE = population_size; // population size
    uint32_t MAX_GEN = max_generation; // max number of generation
    const uint32_t GEN_LOG_ITER = 1000;
    const double MUT_PROB = 0.2;
    const uint32_t ELITE = 20;

    NS_LOG_UNCOND("TSP Deadline Based GA");

    std::mt19937 gen(1337); //! MAGIC NUMBER - Fixed seed

    std::vector<uint32_t> baseOrder;
    for (size_t i = 0; i < points.size(); i++) {
        if (deadlines[i].size() > 0) { // this node have some deadlines
            baseOrder.push_back(i);
        }
    }

    if (baseOrder.size() == 0) {
        return baseOrder;
    }

    std::vector<TSPSolution> population(POP_SIZE);
    for (TSPSolution& sol : population) {
        sol.order = baseOrder;
        std::shuffle(sol.order.begin(), sol.order.end(), gen);
        sol.cost = ComputeDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed);
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
            sol.cost = ComputeDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed);
            new_population.push_back(sol);
        }
        std::swap(population, new_population); // faster than assignment
        std::sort(population.begin(), population.end());
        if (generation % GEN_LOG_ITER == 0) {
            NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
        }
    }
    if (best_cost != nullptr) {
        *best_cost = population[0].cost;
    }
    return population[0].order;

}

std::vector<uint32_t> TSPDeadlineBasedTwoOptOptimize(
    const std::vector<point2D>& points,
    const std::vector<std::vector<double>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const std::vector<uint32_t>& baseOrder,
    uint32_t* best_cost = nullptr
) {
    if (baseOrder.size() <= 1) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    uint32_t bestCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed);
    bool improve = true;

    while (improve) {
        improve = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 1; j < order.size(); ++j) {
                std::reverse(order.begin() + i, order.begin() + j + 1);
                uint32_t newCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed);
                if (newCost < bestCost) {
                    bestCost = newCost;
                    improve = true;
                }
                else {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                }
                std::swap(order[i], order[j]);
                newCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed);
                if (newCost < bestCost) {
                    bestCost = newCost;
                    improve = true;
                }
                else {
                    std::swap(order[i], order[j]);
                }
            }
        }
    }
    if (best_cost != nullptr) {
        *best_cost = bestCost;
    }
    return order;
}

std::vector<uint32_t> TSPDeadlineHelper(
    const std::vector<point2D>& points,
    const std::vector<std::vector<double>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    uint32_t* best_cost = nullptr,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000
) {
    std::vector<uint32_t> order = TSPDeadlineBasedGA(points, deadlines, starting_pos, starting_time, speed, population_size, max_generation);
    order = TSPDeadlineBasedTwoOptOptimize(points, deadlines, starting_pos, starting_time, speed, order, best_cost);
    return order;
}
#pragma endregion

#endif
