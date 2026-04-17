#ifndef KMEAN_H
#define KMEAN_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include "ns3/core-module.h"

#include "datatypes.h"
#include "data-structure-helper.h"
#include "tsp-helper.h"

point2D getCentroid(const std::vector<point2D>& points) {
    double x = 0, y = 0;
    for (const auto& p : points) {
        x += p.x;
        y += p.y;
    }
    return { x / points.size(), y / points.size() };
}

point2D getCentroid(const std::vector<point2D>& points, const std::vector<uint32_t>& index, const point2D current) {
    if (index.size() == 0) return current; // fallback
    double x = 0, y = 0;
    for (const auto& i : index) {
        x += points[i].x;
        y += points[i].y;
    }
    return { x / index.size(), y / index.size() };
}

double getSumInterClusterDistance(const std::vector<point2D>& points) {
    if (points.size() < 2) return 0;
    double distance = 0;
    for (uint32_t i = 0; i < points.size(); i++) {
        for (uint32_t j = i + 1; j < points.size(); j++) {
            distance += dist(points[i], points[j]);
        }
    }
    return distance;
}

double getSumInterClusterDistance(const std::vector<uint32_t>& cluster, const std::vector<point2D>& points) {
    if (cluster.size() < 2) return 0;
    double distance = 0;
    for (uint32_t i : cluster) {
        for (uint32_t j : cluster) {
            distance += dist(points[i], points[j]);
        }
    }
    return distance / 2.0;
}

double getAverageInterClusterDistance(const std::vector<point2D>& points) {
    if (points.size() < 2) return 0;
    double distance = getSumInterClusterDistance(points);
    return distance / (points.size() * (points.size() - 1));
}

double getAverageInterClusterDistance(const std::vector<uint32_t>& cluster, const std::vector<point2D>& points) {
    if (cluster.size() < 2) return 0;
    double distance = getSumInterClusterDistance(cluster, points);
    return distance / (cluster.size() * (cluster.size() - 1));
}

double getClusterRadius(const std::vector<point2D>& points, const point2D& centroid) {
    double radius = 0;
    for (const auto& p : points) {
        radius = std::max(radius, dist(p, centroid));
    }
    return radius;
}

double getClusterRadius(const std::vector<uint32_t>& cluster, const std::vector<point2D>& points, const point2D& centroid) {
    double radius = 0;
    for (uint32_t i : cluster) {
        radius = std::max(radius, dist(points[i], centroid));
    }
    return radius;
}

typedef std::vector<std::vector<uint32_t>> ClusterSolution;

namespace Clustering { // TODO Implement more
    struct ClusterData {
        std::vector<uint32_t> mask;
        uint32_t clusterCount = 0;
        void Init(uint32_t n) {

            mask = std::vector<uint32_t>(n, 0);
        }
    } clusterData;


    ClusterSolution KMeans(const std::vector<point2D>& points, uint32_t k) {
        std::vector<std::vector<uint32_t>> clusters(k);
        if (k == 1) {
            for (uint32_t i = 0; i < points.size(); i++) {
                clusters[0].push_back(i);
            }
            return clusters;
        }

        std::vector<point2D> centroids(k);

        // select initial centroids
        centroids[0] = points[rand() % points.size()];
        for (uint32_t i = 1; i < k; i++) {
            double maxDist = 0;
            uint32_t maxIdx = 0;
            for (uint32_t j = 0; j < points.size(); j++) {
                double totalDist = 0;
                for (point2D& c : centroids) {
                    totalDist += dist(points[j], c);
                    if (totalDist > maxDist) {
                        maxDist = totalDist;
                        maxIdx = j;
                    }
                }
            }
            centroids[i] = points[maxIdx];
        }

        //apply k-means
        const uint32_t MAX_ITER = 1000;
        for (uint32_t iter = 0; iter < MAX_ITER; iter++) {
            // reset clusters
            clusters.clear();
            clusters.resize(k);

            // assign cluster
            for (uint32_t i = 0; i < points.size(); i++) {
                double minDist = dist(points[i], centroids[0]);
                uint32_t minIdx = 0;
                for (uint32_t j = 1; j < k; j++) {
                    double distance = dist(points[i], centroids[j]);
                    if (distance < minDist) {
                        minDist = distance;
                        minIdx = j;
                    }
                }
                clusters[minIdx].push_back(i);
            }

            // recalculate centroid
            for (uint32_t i = 0; i < k; i++) {
                centroids[i] = getCentroid(points, clusters[i], centroids[i]);
            }

            // reassign for empty cluster
            bool has_empty_cluster = false;
            for (uint32_t i = 0; i < k; i++) {
                if (clusters[i].size() > 0)
                    continue;
                has_empty_cluster = true;

                // find biggest
                uint32_t biggest_cluster = 0;
                double max_dist = 0;
                for (uint32_t j = 0; j < k; j++) {
                    if (max_dist < getSumInterClusterDistance(clusters[j], points)) {
                        biggest_cluster = j;
                    }
                }
                // find point farthest from centroid
                max_dist = 0;
                for (auto node : clusters[biggest_cluster]) {
                    if (max_dist < dist(points[node], centroids[biggest_cluster])) {
                        max_dist = dist(points[node], centroids[biggest_cluster]);
                        centroids[i] = points[node];
                    }
                }
                break;
            }

            if (has_empty_cluster && iter >= MAX_ITER - 1) {
                iter--; // continue the loop
            }
        }
        return clusters;
    }

    ClusterSolution BisectingKMeans(const std::vector<point2D>& points, uint32_t k) {
        std::vector<std::vector<uint32_t>> clusters(k);
        return clusters;
    }

    struct TABSplitData {
        std::vector<uint32_t> index1;
        std::vector<uint32_t> index2;
        uint32_t k1, k2;
        double fit;
        bool operator<(const TABSplitData& other) const {
            return fit < other.fit;
        }
    };

    TABSplitData TABsplit(const std::vector<point2D>& points, const std::vector<uint32_t>& indexes, uint32_t n1, uint32_t n2, uint32_t k1, uint32_t k2, uint32_t axis) {
        TABSplitData splitResult;
        double splitThreshold;
        double lowerBound = 0;
        double upperBound = config.areaWidth;
        while (true) { // split so that the lower first group have the same node count as n1
            uint32_t count = 0;
            splitThreshold = (lowerBound + upperBound) / 2;
            for (uint32_t idx : indexes) {
                if (axis == 0 && points[idx].x < splitThreshold) {
                    count++;
                }
                if (axis == 1 && points[idx].y < splitThreshold) {
                    count++;
                }
            }
            if (count == n1)
                break;
            if (count < n1) {
                lowerBound = splitThreshold;
            }
            else {
                upperBound = splitThreshold;
            }
        }
        for (uint32_t idx : indexes) {
            if (axis == 0)
                if (points[idx].x < splitThreshold)
                    splitResult.index1.push_back(idx);
                else
                    splitResult.index2.push_back(idx);
            if (axis == 1)
                if (points[idx].y < splitThreshold)
                    splitResult.index1.push_back(idx);
                else
                    splitResult.index2.push_back(idx);
        }
        auto route1 = TSPHelper(points, DataStructureHelper::GetReversedSet(splitResult.index1, points.size()), 50, 100);
        auto route2 = TSPHelper(points, DataStructureHelper::GetReversedSet(splitResult.index2, points.size()), 50, 100);
        double tspCost1 = ComputeTSPCost(route1, points) / k1;
        double tspCost2 = ComputeTSPCost(route2, points) / k2;
        // TODO TSP Aided Bisect fit formular
        splitResult.fit = std::max(tspCost1, tspCost2) + 0.05 * (tspCost1 + tspCost2);
        splitResult.k1 = k1;
        splitResult.k2 = k2;
        return splitResult;
    }
    void TABReccursive(const std::vector<point2D>& points, std::vector<uint32_t>& indexes, uint32_t k) {
        if (k == 1) {
            for (uint32_t idx : indexes) {
                clusterData.mask[idx] = clusterData.clusterCount;
            }
            clusterData.clusterCount++;
            return;
        }
        if (k == indexes.size()) {
            for (uint32_t idx : indexes) {
                clusterData.mask[idx] = clusterData.clusterCount;
                clusterData.clusterCount++;
            }
            return;
        }
        uint32_t n = indexes.size();
        uint32_t k1 = k / 2;
        uint32_t k2 = k - k1;
        uint32_t n1 = n / k * k1;
        uint32_t n2 = n / k * k2;
        uint32_t residual = n - n1 - n2;
        if (residual > 0) {
            n1 += residual / 2;
            n2 += residual / 2 + residual % 2;
        }
        bool is_4_split = (k1 != k2 || n1 != n2);
        std::vector<TABSplitData> splitOptions;
        splitOptions.reserve(4);
        splitOptions.push_back(TABsplit(points, indexes, n1, n2, k1, k2, 0));
        splitOptions.push_back(TABsplit(points, indexes, n1, n2, k1, k2, 1));
        if (is_4_split) {
            splitOptions.push_back(TABsplit(points, indexes, n2, n1, k2, k1, 0));
            splitOptions.push_back(TABsplit(points, indexes, n2, n1, k2, k1, 1));
        }
        auto bestSplit = std::min_element(splitOptions.begin(), splitOptions.end());
        TABReccursive(points, bestSplit->index1, bestSplit->k1);
        TABReccursive(points, bestSplit->index2, bestSplit->k2);
    }

    ClusterSolution TSPAidedBisect(std::vector<point2D>& points, uint32_t k) {

        uint32_t n = points.size();
        clusterData.Init(n);
        auto allIndex = DataStructureHelper::GetIndexVector(n);
        TABReccursive(points, allIndex, k);
        std::vector<std::vector<uint32_t>> clusters(k);
        for (uint32_t i = 0; i < n; i++) {
            clusters[clusterData.mask[i]].push_back(i);
        }
        return clusters;
    }

    // BMTC: Balanced mTSP with center
    struct BMTC_GA_Chromosome {
        std::vector<uint32_t> mask;
        uint32_t clusterCount = 1;
        double cost;
        bool operator<(const BMTC_GA_Chromosome& other) const {
            return cost < other.cost;
        }
        ClusterSolution GetCluster() const {
            ClusterSolution clusters(clusterCount);
            for (uint32_t i = 0; i < mask.size(); i++) {
                clusters[mask[i]].push_back(i);
            }
            return clusters;
        }

        static std::pair<BMTC_GA_Chromosome, BMTC_GA_Chromosome> Crossover(BMTC_GA_Chromosome p1, BMTC_GA_Chromosome p2) {
            uint32_t n = p1.mask.size();

            for (uint32_t i = 0; i < n; i++) {
                if (rand() % 2 == 0) {
                    std::swap(p1.mask[i], p2.mask[i]);
                }
            }
            return { p1, p2 };
        }
        static BMTC_GA_Chromosome Mutate(BMTC_GA_Chromosome p, double mutateProb = 0.2, uint32_t baseProb = 1, uint32_t bonusProb = 5) {
            std::vector<uint32_t> maskProb = std::vector<uint32_t>(p.clusterCount, bonusProb);
            for (uint32_t i = 0; i < p.mask.size(); i++) {
                maskProb[p.mask[i]] = baseProb; // Nếu cluster chưa được gán bất kì nút nào -> tăng tỉ lệ
            }
            uint32_t totalMaskProb = std::accumulate(maskProb.begin(), maskProb.end(), 0);

            for (uint32_t i = 0; i < p.mask.size(); i++) {

                if (rand() % 100 < mutateProb * 100) {
                    uint32_t prob = rand() % totalMaskProb;
                    for (uint32_t j = 0; j < maskProb.size(); j++) {
                        prob -= maskProb[j];
                        if (prob <= 0) {
                            p.mask[i] = j;
                            break;
                        }
                    }
                }
            }
            return p;
        }
    };

    struct BMTC_TwoPart_Chromosome {
        std::vector<uint32_t> permutation; // Thứ tự
        std::vector<uint32_t> routeLengths; // Số điểm cho mỗi uav
        uint32_t clusterCount = 1;
        double cost;

        bool operator<(const BMTC_TwoPart_Chromosome& other) const {
            return cost < other.cost;
        }

        ClusterSolution GetCluster() const {
            ClusterSolution clusters(clusterCount);
            uint32_t idx = 0;
            for (uint32_t i = 0; i < clusterCount; i++) {
                for (uint32_t j = 0; j < routeLengths[i]; j++) {
                    clusters[i].push_back(permutation[idx++]);
                }
            }
            return clusters;
        }

        static BMTC_TwoPart_Chromosome RandomSample(uint32_t n, uint32_t k) {
            BMTC_TwoPart_Chromosome chromosome;

            chromosome.clusterCount = k;
            chromosome.routeLengths = std::vector<uint32_t>(k, 1);
            chromosome.permutation = DataStructureHelper::GetShuffleIndexVector(n);
            int remaining_points = n - k;
            for (int r = 0; r < remaining_points; r++) {
                chromosome.routeLengths[rand() % k]++;
            }
            return chromosome;
        }

        static std::pair<BMTC_TwoPart_Chromosome, BMTC_TwoPart_Chromosome> Crossover(const BMTC_TwoPart_Chromosome& p1, const BMTC_TwoPart_Chromosome& p2) {
            uint32_t n = p1.permutation.size();
            uint32_t k = p1.routeLengths.size();

            BMTC_TwoPart_Chromosome c1 = p1;
            BMTC_TwoPart_Chromosome c2 = p2;

            // part 1 order crossover 
            uint32_t start = rand() % n;
            uint32_t end = rand() % n;
            if (start > end) std::swap(start, end);

            auto applyOX = [&](const std::vector<uint32_t>& parent1, const std::vector<uint32_t>& parent2, std::vector<uint32_t>& child) {
                std::vector<bool> in_child(n, false);

                // Copy đoạn gene ở giữa từ parent1 sang child
                for (uint32_t i = start; i <= end; i++) {
                    child[i] = parent1[i];
                    in_child[parent1[i]] = true;
                }

                // Điền các phần tử còn lại từ parent2 (bắt đầu từ vị trí end + 1)
                uint32_t curr_idx = (end + 1) % n;
                for (uint32_t i = 0; i < n; i++) {
                    uint32_t p2_idx = (end + 1 + i) % n;
                    if (!in_child[parent2[p2_idx]]) {
                        child[curr_idx] = parent2[p2_idx];
                        curr_idx = (curr_idx + 1) % n;
                    }
                }
                };

            applyOX(p1.permutation, p2.permutation, c1.permutation);
            applyOX(p2.permutation, p1.permutation, c2.permutation);


            // part 2 REPAIRED 1-POINT CROSSOVER
            if (k > 1) { // Chỉ lai ghép routeLengths nếu có nhiều hơn 1 cụm
                uint32_t cut_point = rand() % k;

                auto applyRouteLengthCrossover = [&](const std::vector<uint32_t>& r1, const std::vector<uint32_t>& r2, std::vector<uint32_t>& c_r) {
                    int sum = 0;
                    // Nửa đầu lấy từ r1, nửa sau lấy từ r2
                    for (uint32_t i = 0; i <= cut_point; i++) { c_r[i] = r1[i]; sum += r1[i]; }
                    for (uint32_t i = cut_point + 1; i < k; i++) { c_r[i] = r2[i]; sum += r2[i]; }

                    // Cơ chế Repair (Sửa lỗi) để đảm bảo tổng số điểm bằng n và mỗi xe có >= 1 điểm
                    int diff = static_cast<int>(n) - sum;

                    // Nếu thiếu điểm -> cộng thêm ngẫu nhiên vào các xe
                    while (diff > 0) {
                        c_r[rand() % k]++;
                        diff--;
                    }
                    // Nếu thừa điểm -> trừ bớt ngẫu nhiên (chỉ trừ ở những xe có > 1 điểm)
                    while (diff < 0) {
                        uint32_t idx = rand() % k;
                        if (c_r[idx] > 1) {
                            c_r[idx]--;
                            diff++;
                        }
                    }
                    };

                applyRouteLengthCrossover(p1.routeLengths, p2.routeLengths, c1.routeLengths);
                applyRouteLengthCrossover(p2.routeLengths, p1.routeLengths, c2.routeLengths);
            }

            return { c1, c2 };
        }

        static BMTC_TwoPart_Chromosome Mutate(BMTC_TwoPart_Chromosome p, double mutateProb = 0.2) {
            // Đột biến Phần 1: Swap 2 điểm đến ngẫu nhiên
            if ((rand() % 100) < mutateProb * 100) {
                uint32_t n = p.permutation.size();
                uint32_t idx1 = rand() % n;
                uint32_t idx2 = rand() % n;
                std::swap(p.permutation[idx1], p.permutation[idx2]);
            }

            // Đột biến Phần 2: Chuyển 1 điểm từ uav này sang uav khác
            if ((rand() % 100) < mutateProb * 100) {
                if (p.clusterCount > 1) {
                    uint32_t r1 = rand() % p.clusterCount;
                    uint32_t r2 = rand() % p.clusterCount;
                    // Đảm bảo xe r1 không bị rỗng điểm (để lại ít nhất 1)
                    if (r1 != r2 && p.routeLengths[r1] > 1) {
                        p.routeLengths[r1]--;
                        p.routeLengths[r2]++;
                    }
                }
            }
            return p;
        }
    };


    point2D BMTC_refineHubPosition(const std::vector<point2D>& points, ClusterSolution clusters, const std::vector<double>& cost, point2D basePos, uint32_t ITERATION = 50, double LEARNING_RATE = 0.1) {

        while (ITERATION--) {
            double maxCostAfterInsert = 0;
            uint32_t maxCostIndex = 0;
            // Tìm cluster có khoảng cách xa hub nhất
            for (uint32_t i = 0; i < clusters.size(); i++) {
                if (clusters[i].size() == 0) continue;
                double minInsertCost = std::numeric_limits<double>::max();
                for (uint32_t j = 0; j < clusters[i].size(); j++) {
                    point2D a = points[clusters[i][j]];
                    point2D b = points[clusters[i][(j + 1) % clusters[i].size()]];
                    minInsertCost = std::min(minInsertCost, dist(a, basePos) + dist(b, basePos) - dist(a, b));
                }
                if (cost[i] + minInsertCost > maxCostAfterInsert) {
                    maxCostAfterInsert = cost[i] + minInsertCost;
                    maxCostIndex = i;
                }
            }
            // Kéo hub về
            point2D target = points[clusters[maxCostIndex][rand() % clusters[maxCostIndex].size()]];
            basePos = basePos * (1 - LEARNING_RATE) + target * LEARNING_RATE;
        }
        return basePos;
    }

    std::map<std::set<uint32_t>, std::vector<uint32_t>> BMTC_TSP_routeMap;
    double BMTC_routeLengthCap = 0;
    std::vector<uint32_t> BMTC_TSP_helper(const std::vector<uint32_t>& indexes, const std::vector<point2D>& points) {
        std::set<uint32_t> indexSet(indexes.begin(), indexes.end());
        if (BMTC_TSP_routeMap.find(indexSet) != BMTC_TSP_routeMap.end()) {
            return BMTC_TSP_routeMap[indexSet];
        }
        auto route = TSPTwoOptOptimize(points, indexes);
        BMTC_TSP_routeMap[indexSet] = route;

        return route;
    }

    double BMTC_ComputeCost_wFixedHub(ClusterSolution clusters, const std::vector<point2D>& points, point2D hubPos, double emptyClusterPenalty = 2000) {
        uint32_t validClusters = 0;
        std::vector<double> cost(clusters.size(), 0);

        // compute tsp cost
        for (uint32_t i = 0; i < clusters.size(); i++) {
            if (clusters[i].size() > 0) {
                validClusters++;
                clusters[i] = BMTC_TSP_helper(clusters[i], points);
                cost[i] = ComputeTSPCost(clusters[i], points);
                cost[i] += config.ferrySpeed * config.hoverTime * clusters[i].size();
            }
        }

        // tìm max cost
        double maxCost = 0;
        double minCost = std::numeric_limits<double>::max();
        double totalCost = 0;
        double exceedCap = 0;
        double penalty = 0;
        for (uint32_t i = 0; i < clusters.size(); i++) {
            if (clusters[i].size() == 0) {
                minCost = 0;
                penalty += emptyClusterPenalty;
                continue;
            }
            double minInsertCost = std::numeric_limits<double>::max();
            for (uint32_t j = 0; j < clusters[i].size(); j++) {
                point2D a = points[clusters[i][j]];
                point2D b = points[clusters[i][(j + 1) % clusters[i].size()]];
                minInsertCost = std::min(minInsertCost, dist(a, hubPos) + dist(b, hubPos) - dist(a, b));
            }
            cost[i] += minInsertCost;
            maxCost = std::max(maxCost, cost[i]);
            minCost = std::min(minCost, cost[i]);
            totalCost += cost[i];
            if (cost[i] > BMTC_routeLengthCap) {
                exceedCap += cost[i] - BMTC_routeLengthCap;
            }
        }
        // minimizing maxCost, totalcost, penalty
        // maximizing mincost
        return maxCost + 0.1 * (totalCost + exceedCap) / (double)clusters.size() - 0.01 * minCost + penalty;
    }

    double BMTC_ComputeCost(ClusterSolution clusters, const std::vector<point2D>& points, point2D* returnCentroid = nullptr, bool extraOptimized = false, double emptyClusterPenalty = 2000) {

        uint32_t validClusters = 0;
        point2D cannidateHub = { 0,0 };
        std::vector<double> cost(clusters.size(), 0);

        // compute tsp cost
        for (uint32_t i = 0; i < clusters.size(); i++) {
            cannidateHub = cannidateHub + getCentroid(points, clusters[i], { 0, 0 });
            if (clusters[i].size() > 0) {
                validClusters++;
                // clusters[i] = TSPTwoOptOptimize(points, clusters[i]);
                clusters[i] = BMTC_TSP_helper(clusters[i], points);
                cost[i] = ComputeTSPCost(clusters[i], points);
                cost[i] += config.ferrySpeed * config.hoverTime * clusters[i].size();
            }
        }
        cannidateHub = cannidateHub / validClusters; // center of centers of clusters
        // refine hub position
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 10, 0.005);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 20, 0.002);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 50, 0.001);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 120, 0.0005);
        if (extraOptimized || returnCentroid != nullptr) {
            cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 500, 0.0001);// extra refinement
        }
        if (returnCentroid != nullptr) {
            cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 1000, 0.0001);// extra extra refinement
            *returnCentroid = cannidateHub;
        }
        // tìm max cost
        double maxCost = 0;
        double minCost = std::numeric_limits<double>::max();
        double totalCost = 0;
        double exceedCap = 0;
        double penalty = 0;
        for (uint32_t i = 0; i < clusters.size(); i++) {
            if (clusters[i].size() == 0) {
                minCost = 0;
                penalty += emptyClusterPenalty;
                continue;
            }
            double minInsertCost = std::numeric_limits<double>::max();
            for (uint32_t j = 0; j < clusters[i].size(); j++) {
                point2D a = points[clusters[i][j]];
                point2D b = points[clusters[i][(j + 1) % clusters[i].size()]];
                minInsertCost = std::min(minInsertCost, dist(a, cannidateHub) + dist(b, cannidateHub) - dist(a, b));
            }
            cost[i] += minInsertCost;
            maxCost = std::max(maxCost, cost[i]);
            minCost = std::min(minCost, cost[i]);
            totalCost += cost[i];
            if (cost[i] > BMTC_routeLengthCap) {
                exceedCap += cost[i] - BMTC_routeLengthCap;
            }
        }
        // minimizing maxCost, totalcost, penalty
        // maximizing mincost
        return maxCost + 0.1 * (totalCost + exceedCap) / (double)clusters.size() - 0.01 * minCost + penalty;
    }
    ClusterSolution BMTC_handleEdgeCase(ClusterSolution clusters, const std::vector<point2D>& points, uint32_t k, point2D hubPos) {
        // Tìm empty clusters 
        for (uint32_t i = 0; i < k; i++) {
            if (clusters[i].size() > 0) continue;
            double maxDist = 0;
            uint32_t maxIdx = 0;
            for (auto& cluster : clusters) {
                if (cluster.size() < 2) continue;
                for (auto& pointIdx : cluster) {
                    double distToHub = dist(points[pointIdx], hubPos);
                    if (distToHub > maxDist) {
                        maxDist = distToHub;
                        maxIdx = pointIdx;
                    }
                }
            }
            // gán điểm mới tìm được vào cluster này, và bỏ điểm đó ở cluster cũ
            for (auto& cluster : clusters) {
                for (uint32_t j = 0; j < cluster.size(); j++) {
                    if (cluster[j] == maxIdx) {
                        cluster.erase(cluster.begin() + j);
                        break;
                    }
                }
            }
            clusters[i].push_back(maxIdx);
        }

        return clusters;
    }

    ClusterSolution BMTC_localSearch(const std::vector<point2D>& points, ClusterSolution baseClusters, uint32_t k) {
        bool improved = true;
        ClusterSolution bestClusters = baseClusters;
        double currentBestCost = BMTC_ComputeCost(bestClusters, points);

        while (improved) {
            improved = false;

            // --- MOVE : Di chuyển 1 điểm ---
            for (uint32_t i = 0; i < k && !improved; ++i) {
                for (size_t p_idx = 0; p_idx < bestClusters[i].size(); ++p_idx) {
                    uint32_t pointID = bestClusters[i][p_idx];

                    for (uint32_t j = 0; j < k; ++j) {
                        if (i == j) continue;

                        ClusterSolution temp = bestClusters;
                        temp[i].erase(temp[i].begin() + p_idx);
                        temp[j].push_back(pointID);

                        double newCost = BMTC_ComputeCost(temp, points, nullptr, true);
                        if (newCost < currentBestCost - 1e-6) {
                            currentBestCost = newCost;
                            bestClusters = temp;
                            improved = true;
                            break;
                        }
                    }
                    if (improved) break;
                }
            }

            if (improved) continue; // Nếu Move đã giúp cải thiện, lặp lại vòng lặp chính ngay

            // --- SWAP : Hoán đổi 2 điểm giữa 2 cụm ---
            for (uint32_t i = 0; i < k && !improved; ++i) {
                for (uint32_t j = i + 1; j < k && !improved; ++j) {
                    for (size_t p1_idx = 0; p1_idx < bestClusters[i].size(); ++p1_idx) {
                        for (size_t p2_idx = 0; p2_idx < bestClusters[j].size(); ++p2_idx) {

                            ClusterSolution temp = bestClusters;
                            std::swap(temp[i][p1_idx], temp[j][p2_idx]);

                            double newCost = BMTC_ComputeCost(temp, points, nullptr, true);
                            if (newCost < currentBestCost - 1e-6) {
                                currentBestCost = newCost;
                                bestClusters = temp;
                                improved = true;
                                break;
                            }
                        }
                        if (improved) break;
                    }
                }
            }
        }
        NS_LOG_UNCOND("Balanced mTSP with Center Clustering - LS - best cost: " << currentBestCost);
        return bestClusters;
    }

    ClusterSolution BMTC_localSearch_fixedHub(const std::vector<point2D>& points, ClusterSolution baseClusters, uint32_t k, point2D hubPos) {
        bool improved = true;
        ClusterSolution bestClusters = baseClusters;
        double currentBestCost = BMTC_ComputeCost_wFixedHub(bestClusters, points, hubPos);

        while (improved) {
            improved = false;

            // --- MOVE : Di chuyển 1 điểm ---
            for (uint32_t i = 0; i < k && !improved; ++i) {
                for (size_t p_idx = 0; p_idx < bestClusters[i].size(); ++p_idx) {
                    uint32_t pointID = bestClusters[i][p_idx];

                    for (uint32_t j = 0; j < k; ++j) {
                        if (i == j) continue;

                        ClusterSolution temp = bestClusters;
                        temp[i].erase(temp[i].begin() + p_idx);
                        temp[j].push_back(pointID);

                        double newCost = BMTC_ComputeCost_wFixedHub(temp, points, hubPos);
                        if (newCost < currentBestCost - 1e-6) {
                            currentBestCost = newCost;
                            bestClusters = temp;
                            improved = true;
                            break;
                        }
                    }
                    if (improved) break;
                }
            }

            if (improved) continue; // Nếu Move đã giúp cải thiện, lặp lại vòng lặp chính ngay

            // --- SWAP : Hoán đổi 2 điểm giữa 2 cụm ---
            for (uint32_t i = 0; i < k && !improved; ++i) {
                for (uint32_t j = i + 1; j < k && !improved; ++j) {
                    for (size_t p1_idx = 0; p1_idx < bestClusters[i].size(); ++p1_idx) {
                        for (size_t p2_idx = 0; p2_idx < bestClusters[j].size(); ++p2_idx) {

                            ClusterSolution temp = bestClusters;
                            std::swap(temp[i][p1_idx], temp[j][p2_idx]);

                            double newCost = BMTC_ComputeCost(temp, points, nullptr, true);
                            if (newCost < currentBestCost - 1e-6) {
                                currentBestCost = newCost;
                                bestClusters = temp;
                                improved = true;
                                break;
                            }
                        }
                        if (improved) break;
                    }
                }
            }
        }
        NS_LOG_UNCOND("Balanced mTSP with Center Clustering - LS - best cost: " << currentBestCost);
        return bestClusters;
    }

    ClusterSolution BalancedMT_wCenterClustering_GA(const std::vector<point2D>& points, const uint32_t k, const double expectedRouteLengthCap, uint32_t population_size = 200, uint32_t max_generation = 5000) {
        uint32_t LOG_ITERATION = 1000;
        BMTC_routeLengthCap = expectedRouteLengthCap;
        NS_LOG_UNCOND("Balanced mTSP with Center Clustering - GA");
        // generate initial population
        std::vector<BMTC_GA_Chromosome> population(population_size);
        for (uint32_t i = 0; i < population_size; i++) {
            for (uint32_t j = 0; j < points.size(); j++) {
                population[i].mask.push_back(rand() % k);
            }
            population[i].clusterCount = k;
            population[i].cost = BMTC_ComputeCost(population[i].GetCluster(), points);
        }
        std::sort(population.begin(), population.end());

        for (uint32_t generation = 0; generation < max_generation + 1; generation++) {
            std::vector<BMTC_GA_Chromosome> new_population;
            for (uint32_t i = 0; i < population_size / 2; i++) {
                auto [p1, p2] = BMTC_GA_Chromosome::Crossover(population[i], population[i + 1]);
                p1 = BMTC_GA_Chromosome::Mutate(p1);
                p2 = BMTC_GA_Chromosome::Mutate(p2);
                p1.cost = BMTC_ComputeCost(p1.GetCluster(), points);
                p2.cost = BMTC_ComputeCost(p2.GetCluster(), points);
                new_population.push_back(p1);
                new_population.push_back(p2);
            }
            population.insert(population.end(), new_population.begin(), new_population.end());
            std::sort(population.begin(), population.end());
            population.resize(population_size);
            if (generation % LOG_ITERATION == 0) {
                NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
            }
        }

        point2D hubPos;
        BMTC_ComputeCost(population[0].GetCluster(), points, &hubPos);

        auto clusters = BMTC_handleEdgeCase(population[0].GetCluster(), points, k, hubPos);
        clusters = BMTC_localSearch(points, clusters, k);
        return clusters;
    }

    ClusterSolution BalancedMT_wCenterClustering_GA_v2(const std::vector<point2D>& points, const uint32_t k, const double expectedRouteLengthCap, uint32_t population_size = 200, uint32_t max_generation = 5000) {
        uint32_t LOG_ITERATION = 1000;
        BMTC_routeLengthCap = expectedRouteLengthCap;
        NS_LOG_UNCOND("Balanced mTSP with Center Clustering - GA v2");
        // generate initial population
        std::vector<BMTC_TwoPart_Chromosome> population(population_size);
        for (uint32_t i = 0; i < population_size; i++) {
            population[i] = BMTC_TwoPart_Chromosome::RandomSample(points.size(), k);
            population[i].cost = BMTC_ComputeCost(population[i].GetCluster(), points);
        }

        std::sort(population.begin(), population.end());

        for (uint32_t generation = 0; generation < max_generation + 1; generation++) {
            std::vector<BMTC_TwoPart_Chromosome> new_population;
            for (uint32_t i = 0; i < population_size / 2; i++) {
                auto [p1, p2] = BMTC_TwoPart_Chromosome::Crossover(population[i], population[i + 1]);
                p1 = BMTC_TwoPart_Chromosome::Mutate(p1);
                p2 = BMTC_TwoPart_Chromosome::Mutate(p2);
                p1.cost = BMTC_ComputeCost(p1.GetCluster(), points);
                p2.cost = BMTC_ComputeCost(p2.GetCluster(), points);
                new_population.push_back(p1);
                new_population.push_back(p2);
            }
            population.insert(population.end(), new_population.begin(), new_population.end());
            std::sort(population.begin(), population.end());
            population.resize(population_size);
            if (generation % LOG_ITERATION == 0) {
                NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
            }
        }

        point2D hubPos;
        BMTC_ComputeCost(population[0].GetCluster(), points, &hubPos);

        auto clusters = BMTC_handleEdgeCase(population[0].GetCluster(), points, k, hubPos);
        clusters = BMTC_localSearch(points, clusters, k);
        return clusters;
    }
    ClusterSolution BalancedMT_fixedHub_GA(const std::vector<point2D>& points, const uint32_t k, const double expectedRouteLengthCap, point2D hubPos, uint32_t population_size = 200, uint32_t max_generation = 5000) {
        uint32_t LOG_ITERATION = 200;
        BMTC_routeLengthCap = expectedRouteLengthCap;
        NS_LOG_UNCOND("Balanced mTSP with Center Clustering - GA");
        // generate initial population
        std::vector<BMTC_GA_Chromosome> population(population_size);
        for (uint32_t i = 0; i < population_size; i++) {
            for (uint32_t j = 0; j < points.size(); j++) {
                population[i].mask.push_back(rand() % k);
            }
            population[i].clusterCount = k;
            population[i].cost = BMTC_ComputeCost_wFixedHub(population[i].GetCluster(), points, hubPos);
        }
        std::sort(population.begin(), population.end());

        for (uint32_t generation = 0; generation < max_generation + 1; generation++) {
            std::vector<BMTC_GA_Chromosome> new_population;
            for (uint32_t i = 0; i < population_size / 2; i++) {
                auto [p1, p2] = BMTC_GA_Chromosome::Crossover(population[i], population[i + 1]);
                p1 = BMTC_GA_Chromosome::Mutate(p1);
                p2 = BMTC_GA_Chromosome::Mutate(p2);
                p1.cost = BMTC_ComputeCost_wFixedHub(p1.GetCluster(), points, hubPos);
                p2.cost = BMTC_ComputeCost_wFixedHub(p2.GetCluster(), points, hubPos);
                new_population.push_back(p1);
                new_population.push_back(p2);
            }
            population.insert(population.end(), new_population.begin(), new_population.end());
            std::sort(population.begin(), population.end());
            population.resize(population_size);
            if (generation % LOG_ITERATION == 0) {
                NS_LOG_UNCOND("generation " + std::to_string(generation) + "   - best cost : " + std::to_string(population[0].cost));
            }
        }

        auto clusters = BMTC_handleEdgeCase(population[0].GetCluster(), points, k, hubPos);
        clusters = BMTC_localSearch(points, clusters, k);
        return clusters;
    }


    struct BMTC_routeExtendSolution {
        std::vector<uint32_t> order;
        double reward = 0;
        bool operator<(const BMTC_routeExtendSolution& other) const {
            return reward < other.reward;
        }
        bool operator>(const BMTC_routeExtendSolution& other) const {
            return reward > other.reward;
        }
        static BMTC_routeExtendSolution swapNeighbor(BMTC_routeExtendSolution solution, uint32_t step = 2) {
            uint32_t n = solution.order.size();
            while (step--) {
                uint32_t i = rand() % n;
                uint32_t j = rand() % n;
                std::swap(solution.order[i], solution.order[j]);
            }
            return solution;
        }
        static BMTC_routeExtendSolution reverseNeighbor(BMTC_routeExtendSolution solution, uint32_t step = 1) {
            uint32_t n = solution.order.size();
            while (step--) {
                uint32_t i = rand() % n;
                uint32_t j = rand() % n;
                if (i > j) std::swap(i, j);
                std::reverse(solution.order.begin() + i, solution.order.begin() + j + 1);
            }
            return solution;
        }

    };
    double BMTC_RE_computeReward(
        const BMTC_routeExtendSolution& solution,
        const std::vector<point2D>& points,
        const ClusterSolution& baseClusters,
        const point2D hubPos,
        const std::vector<double>& rewards,
        const double routelengthCap
    ) {
        double reward = 0;
        std::set<uint32_t> added;
        for (auto& cluster : baseClusters) {
            if (cluster.size() == 0) continue;
            // Tính độ dài lộ trình của cluster, bỏ qua hubpos ở cuối cùng
            double routeLength = dist(hubPos, points[cluster[0]]);
            for (uint32_t i = 0; i < cluster.size() - 1; i++) {
                routeLength += dist(points[cluster[i]], points[cluster[i + 1]]);
                routeLength += config.ferrySpeed * config.hoverTime;
            }

            point2D lastPoint = points[cluster[cluster.size() - 1]];

            // Thêm các điểm một cách tham lam vào cuối lộ trình
            for (auto node : solution.order) {
                if (added.find(node) != added.end()) continue; // nút này đã được phục vụ
                double addCost =
                    dist(lastPoint, points[node]) + dist(points[node], hubPos)
                    + 2 * config.ferrySpeed * config.hoverTime;

                if (routeLength + addCost < routelengthCap - 0.001) {
                    routeLength += dist(lastPoint, points[node]) + config.ferrySpeed * config.hoverTime;
                    lastPoint = points[node];
                    added.insert(node);
                    reward += rewards[node];
                }
            }
        }
        double maxPossibleReward = 0;
        for (auto r : rewards) {
            maxPossibleReward += r;
        }

        return reward / maxPossibleReward;
    }
    ClusterSolution BMTC_RE_extend(
    const BMTC_routeExtendSolution& solution,
    const std::vector<point2D>& points,
    const ClusterSolution& baseClusters,
    const point2D hubPos,
    const double routelengthCap
    ) {
        auto newCluster = baseClusters;
        std::set<uint32_t> added;
        for (auto& cluster : newCluster) {
            if (cluster.size() == 0) continue;
            // Tính độ dài lộ trình của cluster, bỏ qua hubpos ở cuối cùng
            double routeLength = dist(hubPos, points[cluster[0]]) + config.ferrySpeed * config.hoverTime;
            for (uint32_t i = 0; i < cluster.size() - 1; i++) {
                routeLength += dist(points[cluster[i]], points[cluster[i + 1]]);
                routeLength += config.ferrySpeed * config.hoverTime;
            }

            point2D lastPoint = points[cluster[cluster.size() - 1]];

            // Thêm các điểm một cách tham lam vào cuối lộ trình
            for (auto node : solution.order) {
                if (added.find(node) != added.end()) continue; // nút này đã được phục vụ
                double addCost =
                    dist(lastPoint, points[node]) + dist(points[node], hubPos)
                    + 2 * config.ferrySpeed * config.hoverTime;

                if (routeLength + addCost < routelengthCap - 0.001) {
                    routeLength += dist(lastPoint, points[node]) + config.ferrySpeed * config.hoverTime;
                    lastPoint = points[node];
                    added.insert(node);
                    cluster.push_back(node);
                }
            }
        }

        return newCluster;
    }

    // Hàm SA hoàn chỉnh
    ClusterSolution BMTC_routeExtend_SA(
        const std::vector<point2D>& points,
        const ClusterSolution& baseClusters,
        const point2D hubPos,
        // const double expectedRouteLengthCap,
        uint32_t multi_start = 500,
        double starting_temperature = 1.5,
        double cooling = 0.95
    ) {
        uint32_t n = points.size();
        uint32_t k = baseClusters.size();
        if (n == 0) return baseClusters;

        //* --- Tính reward cho từng điểm ---
        std::vector<double> rewards(n, 0);
        double maxRouteLength = 0;
        for (auto cluster : baseClusters) {
            double distance = 0;
            point2D currentPos = hubPos;
            for (auto pIdx : cluster) {
                distance += dist(currentPos, points[pIdx]);
                distance += config.ferrySpeed * config.hoverTime;
                currentPos = points[pIdx];
                rewards[pIdx] = distance; // Tạm đặt reward = khoảng cách từ hub đến nó trong một vòng lộ trình
            }
            distance += dist(currentPos, hubPos);
            distance += config.ferrySpeed * config.hoverTime;
            maxRouteLength = std::max(maxRouteLength, distance);
        }
        std::vector<uint32_t> baseOrder;
        for (uint32_t i = 0; i < n; i++) {
            // Reward = khoảng cách từ một điểm đến hub trong lộ trình ban đầu
            rewards[i] = maxRouteLength / 2.0 - rewards[i];
            if (rewards[i] < 0) rewards[i] = 0; // Chỉ xét các điểm mà khoảng cách từ nó đến hub > maxroute / 2
            else baseOrder.push_back(i);
            if (config.CHUB_squareREReward)
                rewards[i] *= rewards[i];
        }
        NS_LOG_UNCOND("Route length cap: " << maxRouteLength);
        double expectedRouteLengthCap = maxRouteLength;
        if (baseOrder.size() == 0) return baseClusters;

        //* --- Simulated Anealing ---
        std::mt19937 rng(rand());

        BMTC_routeExtendSolution globalBestSolution;
        globalBestSolution.reward = 0;

        while (multi_start--) {
            // Tạo lời giải ban đầu ngẫu nhiên cho mỗi lần start
            BMTC_routeExtendSolution currentSolution;
            currentSolution.order = baseOrder;
            std::shuffle(currentSolution.order.begin(), currentSolution.order.end(), rng);

            // Tính cost ban đầu
            currentSolution.reward = BMTC_RE_computeReward(currentSolution, points, baseClusters, hubPos, rewards, expectedRouteLengthCap);

            BMTC_routeExtendSolution localBest = currentSolution;
            double T = starting_temperature;
            double min_T = 0.001; // Ngưỡng dừng (đóng băng)

            uint32_t iterations_per_temp = baseOrder.size() * k; // Số vòng lặp tại trước khi giảm nhiệt độ (Inner loop)

            // SA
            while (T > min_T) {
                for (uint32_t i = 0; i < iterations_per_temp; i++) {

                    BMTC_routeExtendSolution nextSolution = BMTC_routeExtendSolution::reverseNeighbor(currentSolution, 1);
                    nextSolution.reward = BMTC_RE_computeReward(nextSolution, points, baseClusters, hubPos, rewards, expectedRouteLengthCap);

                    double deltaR = nextSolution.reward - currentSolution.reward;

                    // 1. Nếu Reward tăng (deltaR > 0): Chấp nhận ngay
                    // 2. Nếu Reward giảm (deltaR < 0): Chấp nhận với xác suất Metropolis
                    if (deltaR > 0 || (exp(deltaR / T) > (double)rand() / RAND_MAX)) {
                        currentSolution = nextSolution;
                        if (currentSolution.reward > localBest.reward) {
                            localBest = currentSolution;
                        }
                    }
                }
                T *= cooling;
            }
            // Sau mỗi lần start, cập nhật lời giải tốt nhất toàn cục
            if (localBest.reward > globalBestSolution.reward) {
                globalBestSolution = localBest;
            }
            // NS_LOG_UNCOND("Attemp #" << multi_start << " - Best reward:" << globalBestSolution.reward);
        }
        NS_LOG_UNCOND("Route extend - Best reward:" << globalBestSolution.reward);
        return BMTC_RE_extend(globalBestSolution, points, baseClusters, hubPos, expectedRouteLengthCap);
    }

};
#endif 