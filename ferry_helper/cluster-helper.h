#ifndef KMEAN_H
#define KMEAN_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include "ns3/core-module.h"

#include "datatypes.h"

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
};
#endif // KMEAN_H