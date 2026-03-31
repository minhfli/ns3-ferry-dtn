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

    double BMTC_ComputeCost(ClusterSolution clusters, const std::vector<point2D>& points, point2D* returnCentroid = nullptr, bool extraOptimized = false, double emptyClusterPenalty = 2000) {

        uint32_t validClusters = 0;
        point2D cannidateHub = { 0,0 };
        std::vector<double> cost(clusters.size(), 0);

        // compute tsp cost
        for (uint32_t i = 0; i < clusters.size(); i++) {
            cannidateHub = cannidateHub + getCentroid(points, clusters[i], { 0, 0 });
            if (clusters[i].size() > 0) {
                validClusters++;
                clusters[i] = TSPTwoOptOptimize(points, clusters[i]);
                cost[i] = ComputeTSPCost(clusters[i], points);
                cost[i] += config.ferrySpeed * config.hoverTime * clusters[i].size();
            }
        }
        cannidateHub = cannidateHub / validClusters; // center of centers of clusters
        // refine hub position
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 10, 0.1);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 20, 0.05);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 50, 0.005);
        cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 120, 0.001);
        if (returnCentroid != nullptr) {
            cannidateHub = BMTC_refineHubPosition(points, clusters, cost, cannidateHub, 500, 0.0001);// extra refinement
            *returnCentroid = cannidateHub;
        }
        // tìm max cost
        double maxCost = 0;
        double minCost = std::numeric_limits<double>::max();
        double totalCost = 0;
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

            maxCost = std::max(maxCost, cost[i] + minInsertCost);
            minCost = std::min(minCost, cost[i] + minInsertCost);
            totalCost += cost[i] + minInsertCost;
        }
        // minimizing maxCost, totalcost, penalty
        // maximizing mincost
        return maxCost + 0.05 * totalCost / (double)clusters.size() - 0.05 * minCost + penalty;
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

    ClusterSolution BalancedMT_wCenterClustering_GA(const std::vector<point2D>& points, uint32_t k, uint32_t population_size = 200, uint32_t max_generation = 5000) {
        uint32_t LOG_ITERATION = 200;

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

        return clusters;
    }

    ClusterSolution BMTC_localSearch(const std::vector<point2D>& points, ClusterSolution baseClusters, uint32_t k) {
        return baseClusters;
    }

    ClusterSolution BMTC_routeExtend(const std::vector<point2D>& points, ClusterSolution baseClusters, uint32_t k) { // TODO
        return baseClusters;
    }
};
#endif 