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
#include "data-structure-helper.h"


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
inline double ComputeTSPCost(const std::vector<uint32_t>& order,
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
        sol.cost = ComputeTSPCost(sol.order, points);
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
            sol.cost = ComputeTSPCost(sol.order, points);
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

std::vector<uint32_t> TSPClassicGA(const std::vector<point2D>& points, const std::vector<uint32_t>& included, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
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

    if (included.size() <= 3) {
        return included;
    }

    std::vector<TSPSolution> population(POP_SIZE);
    for (TSPSolution& sol : population) {
        sol.order = included;
        std::shuffle(sol.order.begin(), sol.order.end(), gen);
        sol.cost = ComputeTSPCost(sol.order, points);
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
            sol.cost = ComputeTSPCost(sol.order, points);
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
    if (baseOrder.size() <= 3) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    uint32_t n = order.size();
    // double bestCost = ComputeTSPCost(order, points);
    bool improve = true;
    while (improve) {
        improve = false;
        for (size_t i = 0; i < n - 1; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                if (j - i == n - 1) continue;
                uint32_t prev_i = (i == 0) ? n - 1 : i - 1;
                uint32_t next_j = (j == n - 1) ? 0 : j + 1;
                const auto& node_A = points[order[prev_i]];
                const auto& node_B = points[order[i]];
                const auto& node_C = points[order[j]];
                const auto& node_D = points[order[next_j]];

                double old_edges_cost = dist(node_A, node_B) + dist(node_C, node_D);
                double new_edges_cost = dist(node_A, node_C) + dist(node_B, node_D);

                double deltaCost = new_edges_cost - old_edges_cost;
                // double newCost = ComputeTSPCost(order, points);
                if (deltaCost < -0.00001) {
                    // bestCost += deltaCost;
                    improve = true;
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                }

            }
        }
    }
    return order;
}

std::vector<uint32_t> TSPOrOptOptimize(const std::vector<point2D>& points, const std::vector<uint32_t>& baseOrder) {
    if (baseOrder.size() <= 3) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    bool improve = true;

    // Xét các đoạn có độ dài 3, 2, 1
    std::vector<size_t> segment_lengths = { 3, 2, 1 };

    while (improve) {
        improve = false;
        size_t n = order.size();
        for (size_t len : segment_lengths) {
            if (len >= n) continue;
            for (size_t i = 0; i <= n - len; ++i) {
                for (size_t j = 0; j <= n; ++j) {
                    // Bỏ qua nếu vị trí chèn nằm ngay bên trong hoặc ngay cạnh đoạn đang xét
                    if (j >= i && j <= i + len) continue;

                    // các index liên quan 
                    size_t prev_i = (i == 0) ? n - 1 : i - 1;
                    size_t next_seg = (i + len == n) ? 0 : i + len;

                    // Vị trí chèn j tương đương việc cắt cạnh giữa A và B để nhét đoạn [i, i+len-1] vào
                    size_t A = (j == 0) ? n - 1 : j - 1;
                    size_t B = (j == n) ? 0 : j;

                    // Bỏ qua các phép di chuyển bọc vòng (wrap-around) chỉ làm dịch chuyển mảng 
                    // mà không thay đổi cấu trúc thật của chu trình
                    if (A == prev_i || A == i + len - 1) continue;

                    const auto& node_prev_i = points[order[prev_i]];
                    const auto& node_start = points[order[i]];
                    const auto& node_end = points[order[i + len - 1]];
                    const auto& node_next = points[order[next_seg]];
                    const auto& node_A = points[order[A]];
                    const auto& node_B = points[order[B]];

                    // Tổng khoảng cách 3 cạnh cũ sắp bị đứt
                    double old_cost = dist(node_prev_i, node_start)
                        + dist(node_end, node_next)
                        + dist(node_A, node_B);

                    double new_cost = dist(node_prev_i, node_next)
                        + dist(node_A, node_start)
                        + dist(node_end, node_B);

                    double delta = new_cost - old_cost;

                    // Nếu quãng đường giảm (dùng -1e-7 để tránh sai số dấu phẩy động)
                    if (delta < -1e-7) {
                        // Thực hiện hoán đổi trên mảng
                        std::vector<uint32_t> segment(order.begin() + i, order.begin() + i + len);
                        order.erase(order.begin() + i, order.begin() + i + len);

                        size_t insert_pos = (j > i) ? j - len : j;
                        order.insert(order.begin() + insert_pos, segment.begin(), segment.end());

                        improve = true;
                        break;
                    }
                }
                if (improve) break;
            }
            if (improve) break;
        }
    }
    return order;
}

std::vector<uint32_t> Build3Opt(const std::vector<uint32_t>& order, size_t i, size_t j, size_t k, int case_num) {
    std::vector<uint32_t> newOrder;
    newOrder.reserve(order.size());

    // A: [0, i-1]
    newOrder.insert(newOrder.end(), order.begin(), order.begin() + i);

    std::vector<uint32_t> B(order.begin() + i, order.begin() + j);
    std::vector<uint32_t> C(order.begin() + j, order.begin() + k);

    // Xử lý lật ngược (reverse) tùy theo case_num
    if (case_num == 2 || case_num == 4) std::reverse(C.begin(), C.end());
    if (case_num == 3 || case_num == 4) std::reverse(B.begin(), B.end());

    // Thêm C rồi thêm B
    newOrder.insert(newOrder.end(), C.begin(), C.end());
    newOrder.insert(newOrder.end(), B.begin(), B.end());

    // Đoạn D: [k, end]
    newOrder.insert(newOrder.end(), order.begin() + k, order.end());

    return newOrder;
}
#include <vector>
#include <algorithm>

std::vector<uint32_t> TSPThreeOptOptimize(const std::vector<point2D>& points, const std::vector<uint32_t>& baseOrder) {
    if (baseOrder.size() <= 4) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    bool improve = true;

    while (improve) {
        improve = false;
        size_t n = order.size();

        for (size_t i = 1; i < n - 2; ++i) {
            for (size_t j = i + 1; j < n - 1; ++j) {
                for (size_t k = j + 1; k < n; ++k) {

                    // Xác định 6 điểm tạo nên 3 cạnh bị đứt
                    const auto& A_out = points[order[i - 1]];
                    const auto& B_in = points[order[i]];
                    const auto& B_out = points[order[j - 1]];
                    const auto& C_in = points[order[j]];
                    const auto& C_out = points[order[k - 1]];
                    const auto& D_in = points[order[k]];

                    // Tính tổng 3 cạnh cũ sắp bị đứt
                    double old_cost = dist(A_out, B_in) + dist(B_out, C_in) + dist(C_out, D_in);

                    double best_delta = 0.0;
                    int best_case = 0;

                    // Trường hợp 1: A - C - B - D (Đảo khối B và C)
                    double new_cost_1 = dist(A_out, C_in) + dist(C_out, B_in) + dist(B_out, D_in);
                    double delta_1 = new_cost_1 - old_cost;
                    if (delta_1 < best_delta) { best_delta = delta_1; best_case = 1; }

                    // Trường hợp 2: A - C' - B - D (Khối C đảo ngược)
                    double new_cost_2 = dist(A_out, C_out) + dist(C_in, B_in) + dist(B_out, D_in);
                    double delta_2 = new_cost_2 - old_cost;
                    if (delta_2 < best_delta) { best_delta = delta_2; best_case = 2; }

                    // Trường hợp 3: A - C - B' - D (Khối B đảo ngược)
                    double new_cost_3 = dist(A_out, C_in) + dist(C_out, B_out) + dist(B_in, D_in);
                    double delta_3 = new_cost_3 - old_cost;
                    if (delta_3 < best_delta) { best_delta = delta_3; best_case = 3; }

                    // Trường hợp 4: A - C' - B' - D (Cả B và C đều đảo ngược)
                    double new_cost_4 = dist(A_out, C_out) + dist(C_in, B_out) + dist(B_in, D_in);
                    double delta_4 = new_cost_4 - old_cost;
                    if (delta_4 < best_delta) { best_delta = delta_4; best_case = 4; }

                    if (best_delta < -1e-7) {
                        order = Build3Opt(order, i, j, k, best_case);
                        improve = true;
                        break;
                    }
                }
                if (improve) break;
            }
            if (improve) break;
        }
    }
    return order;
}

std::vector<uint32_t> TSPAllOptOptimize(const std::vector<point2D>& points, const std::vector<uint32_t>& baseOrder) {
    std::vector<uint32_t> currentOrder = baseOrder;
    bool system_improved = true;

    while (system_improved) {
        system_improved = false;

        //  2-opt
        std::vector<uint32_t> order_2opt = TSPTwoOptOptimize(points, currentOrder);
        if (order_2opt != currentOrder) {
            currentOrder = order_2opt;
            system_improved = true;
        }

        //  Or-opt trên kết quả 2-opt
        std::vector<uint32_t> order_oropt = TSPOrOptOptimize(points, currentOrder);
        if (order_oropt != currentOrder) {
            currentOrder = order_oropt;
            system_improved = true;
            // Nếu Or-opt thay đổi cấu trúc, quay lại 2-opt 
            continue;
        }

        // 3-opt 
        std::vector<uint32_t> order_3opt = TSPThreeOptOptimize(points, currentOrder);
        if (order_3opt != currentOrder) {
            currentOrder = order_3opt;
            system_improved = true;
            // 3-opt vừa phá vỡ cấu trúc cũ, quay lại 2-opt để dọn dẹp
            continue;
        }
    }

    return currentOrder;
}
std::vector<uint32_t> TSPHelper(const std::vector<point2D>& points, const std::set<uint32_t>& excludeIdx = {}, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
    std::vector<uint32_t> order = TSPClassicGA(points, excludeIdx, population_size, max_generation);
    order = TSPAllOptOptimize(points, order);
    return order;
}
std::vector<uint32_t> TSPHelperNoTwoOpt(const std::vector<point2D>& points, const std::set<uint32_t>& excludeIdx = {}, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
    std::vector<uint32_t> order = TSPClassicGA(points, excludeIdx, population_size, max_generation);
    return order;
}

std::vector<uint32_t> TSPHelper(const std::vector<point2D>& points, const std::vector<uint32_t>& included, const uint32_t population_size = 100, const uint32_t max_generation = 2000) {
    std::vector<uint32_t> order = TSPClassicGA(points, included, population_size, max_generation);
    order = TSPTwoOptOptimize(points, order);
    return order;
}

FerryRoute TSPTwoOptHelper(FerryRoute route) {
    uint32_t n = route.size();
    if (n <= 3) return route;
    double currentCost = 0;
    for (uint32_t i = 0; i < n; i++) {
        point2D A = route[i].pos;
        point2D B = route[(i + 1) % n].pos;
        currentCost += dist(A, B);
    }
    bool improved = true;
    while (improved) {
        improved = false;
        for (uint32_t i = 0; i < n - 2; i++) {
            for (uint32_t j = i + 2; j < n; j++) {
                // std::reverse(route.begin() + i + 1, route.begin() + j + 1);
                double newCost = currentCost;
                point2D A = route[i].pos;
                point2D B = route[(i + 1) % n].pos;
                point2D C = route[j].pos;
                point2D D = route[(j + 1) % n].pos;
                newCost += dist(A, C) + dist(B, D);
                newCost -= dist(A, B) + dist(C, D);
                if (newCost < currentCost - 0.01) {
                    std::reverse(route.begin() + i + 1, route.begin() + j + 1);
                    currentCost = newCost;
                    improved = true;
                }
            }
        }
    }
    return route;
}


FerryRoute TSPOptHelper(const FerryRoute& route) {
    FerryRoute newRoute;
    uint32_t n = route.size();
    if (n <= 3) return route;
    std::vector<point2D> points(n);
    for (uint32_t i = 0; i < n; i++) {
        points[i] = route[i].pos;
    }
    std::vector<uint32_t> order = DataStructureHelper::GetIndexVector(n);
    order = TSPAllOptOptimize(points, order);

    for (uint32_t i = 0; i < n; i++) {
        newRoute.push_back(route[order[i]]);
    }
    return newRoute;
}
#pragma endregion

#pragma region "DEADLINE TSP"

uint32_t ComputeDeadlineCost(const std::vector<uint32_t>& order,
    const std::vector<point2D>& points,
    const std::vector<std::vector<double>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const double hoverTime
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
        currentTime += hoverTime;

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
    const double hoverTime,
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
        sol.cost = ComputeDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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
            sol.cost = ComputeDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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
    const double hoverTime,
    const std::vector<uint32_t>& baseOrder,
    uint32_t* best_cost = nullptr
) {
    if (baseOrder.size() <= 1) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    uint32_t bestCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
    bool improve = true;

    while (improve) {
        improve = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 1; j < order.size(); ++j) {
                std::reverse(order.begin() + i, order.begin() + j + 1);
                uint32_t newCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
                if (newCost < bestCost) {
                    bestCost = newCost;
                    improve = true;
                }
                else {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                }
                std::swap(order[i], order[j]);
                newCost = ComputeDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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
    const double hoverTime,
    uint32_t* best_cost = nullptr,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000
) {
    std::vector<uint32_t> order = TSPDeadlineBasedGA(points, deadlines, starting_pos, starting_time, speed, hoverTime, population_size, max_generation);
    order = TSPDeadlineBasedTwoOptOptimize(points, deadlines, starting_pos, starting_time, speed, hoverTime, order, best_cost);
    return order;
}
#pragma endregion

#pragma region "WEIGHTED DEADLINE"
double ComputeWeightedDeadlineCost(const std::vector<uint32_t>& order,
    const std::vector<point2D>& points,
    const std::vector<std::vector<WeightedDeadline>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const double hoverTime
) {
    double cost = 0;
    double currentTime = starting_time;
    point2D currentPos = starting_pos;
    for (size_t i = 0; i < order.size(); ++i) {
        point2D nextPos = points[order[i]];
        double distance = dist(currentPos, nextPos);
        double travelTime = distance / speed;
        currentTime += travelTime;

        // check deadlines
        for (auto deadline : deadlines[order[i]]) {
            if (currentTime <= deadline.time) {
                cost -= deadline.weight; // reward
            }
        }
        currentTime += hoverTime;
        currentPos = nextPos;
    }
    // NS_LOG_UNCOND("Hello?");

    return cost;
}
std::vector<uint32_t> TSPWeightedDeadlineGA(
    const std::vector<point2D>& points,
    const std::vector<std::vector<WeightedDeadline>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const double hoverTime,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000,
    double* best_cost = nullptr
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
        sol.cost = ComputeWeightedDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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
            sol.cost = ComputeWeightedDeadlineCost(sol.order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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

std::vector<uint32_t> TSPWeightedDeadlineTwoOpt(
    const std::vector<point2D>& points,
    const std::vector<std::vector<WeightedDeadline>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const double hoverTime,
    const std::vector<uint32_t>& baseOrder,
    double* best_cost = nullptr
) {
    if (baseOrder.size() <= 1) {
        return baseOrder;
    }

    std::vector<uint32_t> order = baseOrder;
    double bestCost = ComputeWeightedDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
    bool improve = true;

    while (improve) {
        improve = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 1; j < order.size(); ++j) {
                std::reverse(order.begin() + i, order.begin() + j + 1);
                double newCost = ComputeWeightedDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
                if (newCost < bestCost - 0.001) {
                    bestCost = newCost;
                    improve = true;
                }
                else {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                }
                std::swap(order[i], order[j]);
                newCost = ComputeWeightedDeadlineCost(order, points, deadlines, starting_pos, starting_time, speed, hoverTime);
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

std::vector<uint32_t> TSPWeightedDeadlineHelper(
    const std::vector<point2D>& points,
    const std::vector<std::vector<WeightedDeadline>>& deadlines,
    const point2D starting_pos,
    const double starting_time,
    const double speed,
    const double hoverTime,
    double* best_cost = nullptr,
    const uint32_t population_size = 100,
    const uint32_t max_generation = 2000
) {
    std::vector<uint32_t> order = TSPWeightedDeadlineGA(points, deadlines, starting_pos, starting_time, speed, hoverTime, population_size, max_generation);
    order = TSPWeightedDeadlineTwoOpt(points, deadlines, starting_pos, starting_time, speed, hoverTime, order, best_cost);
    return order;
}


#pragma endregion

#endif
