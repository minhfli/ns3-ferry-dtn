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
    }
    return clusters;
}


#endif // KMEAN_H