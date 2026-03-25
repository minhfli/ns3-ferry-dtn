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

namespace Clustering { // TODO Implement more
    struct ClusterData {
        std::vector<uint32_t> mask;
        uint32_t clusterCount = 0;
        void Init(uint32_t n) {

            mask = std::vector<uint32_t>(n, 0);
        }
    } clusterData;


    std::vector<std::vector<uint32_t>> KMeans(const std::vector<point2D>& points, uint32_t k) {
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

    std::vector<std::vector<uint32_t>> BisectingKMeans(const std::vector<point2D>& points, uint32_t k) {
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
        double tspCost1 = ComputeCost(route1, points) / k1;
        double tspCost2 = ComputeCost(route2, points) / k2;
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

    std::vector<std::vector<uint32_t>> TSPAidedBisect(std::vector<point2D>& points, uint32_t k) {

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
};
#endif // KMEAN_H