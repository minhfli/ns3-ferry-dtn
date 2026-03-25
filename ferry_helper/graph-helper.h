#ifndef GRAPH_HELPER_H
#define GRAPH_HELPER_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "global.h"
#include "datatypes.h"
#include "data-structure-helper.h"

struct Graph {
    uint32_t n;
    std::vector<point2D> points;
    std::vector<std::vector<uint32_t>> adjacent;
    double cost = 0;
};

/**
 * Gabriel Graph - O(n^3)
 * Một cạnh (i, j) tồn tại nếu đường tròn nhận đoạn thẳng ij làm đường kính không chứa bất kỳ điểm s nào khác bên trong.
 * dist(i, j)^2 <= dist(i, s)^2 + dist(j, s)^2 với mọi s.
 */
Graph BuildGabrielGraph(std::vector<point2D>& points) {
    uint32_t n = static_cast<uint32_t>(points.size());
    Graph g;
    g.n = n;
    g.points = points;
    g.adjacent.resize(n);

    if (n < 2) return g;

    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t j = i + 1; j < n; ++j) {
            double d2_ij = distSq(points[i], points[j]);
            bool is_gabriel = true;

            for (uint32_t s = 0; s < n; ++s) {
                if (s == i || s == j) continue;

                double d2_is = distSq(points[i], points[s]);
                double d2_js = distSq(points[j], points[s]);

                if (d2_is + d2_js < d2_ij - 1e-9) {
                    is_gabriel = false;
                    break;
                }
            }

            if (is_gabriel) {
                g.adjacent[i].push_back(j);
                g.adjacent[j].push_back(i);
            }
        }
    }
    return g;
}

// --- Hàm tính đường kính (Diameter) dựa trên số bước nhảy ---
double calculateDiameter(const Graph& g) {
    uint32_t n = g.n;
    if (n <= 1) return 0.0;
    // Initialize
    const double INF = std::numeric_limits<double>::max();
    std::vector<std::vector<double>> d(n, std::vector<double>(n, INF));

    for (uint32_t i = 0; i < n; ++i) d[i][i] = 0.0;
    for (uint32_t u = 0; u < n; ++u) {
        for (uint32_t v : g.adjacent[u]) {
            d[u][v] = dist(g.points[u], g.points[v]);
        }
    }
    // Floyd-Warshall 
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t i = 0; i < n; ++i) {
            for (uint32_t j = 0; j < n; ++j) {
                if (d[i][k] != INF && d[k][j] != INF) {
                    d[i][j] = std::min(d[i][j], d[i][k] + d[k][j]);
                }
            }
        }
    }
    // Find max distance
    double max_dist = 0.0;
    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t j = i + 1; j < n; ++j) {
            if (d[i][j] != INF) {
                max_dist = std::max(max_dist, d[i][j]);
            }
        }
    }
    return max_dist;
}

/**
 * 1-CENTER (STAR GRAPH)
 */
Graph BuildOneCenterGraph(const std::vector<point2D>& points) {
    uint32_t n = points.size();
    double best_cost = std::numeric_limits<double>::max();
    Graph best_graph;
    best_graph.n = n;
    best_graph.points = points;

    for (uint32_t c = 0; c < n; ++c) {
        Graph g; g.n = n; g.points = points; g.adjacent.resize(n);
        for (uint32_t i = 0; i < n; ++i) {
            if (i != c) {
                g.adjacent[c].push_back(i);
                g.adjacent[i].push_back(c);
            }
        }

        double diam = calculateDiameter(g);
        double current_cost = diam; // Cost = Diameter

        if (current_cost < best_cost) {
            best_cost = current_cost;
            best_graph.adjacent = g.adjacent;
            best_graph.cost = best_cost;
        }
    }
    return best_graph;
}

/**
 *  2-CENTER (DOUBLE STAR)
 */
Graph BuildTwoCenterGraph(const std::vector<point2D>& points) {
    uint32_t n = points.size();
    double best_cost = std::numeric_limits<double>::max();
    Graph best_graph;
    best_graph.n = n;
    best_graph.points = points;

    if (n < 2) return BuildOneCenterGraph(points);

    for (uint32_t c1 = 0; c1 < n; ++c1) {
        for (uint32_t c2 = c1 + 1; c2 < n; ++c2) {
            Graph g; g.n = n; g.points = points; g.adjacent.resize(n);
            g.adjacent[c1].push_back(c2);
            g.adjacent[c2].push_back(c1);

            double total_edge_len = dist(points[c1], points[c2]);

            for (uint32_t i = 0; i < n; ++i) {
                if (i == c1 || i == c2) continue;
                double d1 = dist(points[i], points[c1]);
                double d2 = dist(points[i], points[c2]);

                if (d1 < d2) {
                    g.adjacent[c1].push_back(i);
                    g.adjacent[i].push_back(c1);
                    total_edge_len += d1;
                }
                else {
                    g.adjacent[c2].push_back(i);
                    g.adjacent[i].push_back(c2);
                    total_edge_len += d2;
                }
            }

            double diam = calculateDiameter(g);
            double current_cost = 0.8 * diam + 0.2 * total_edge_len;

            if (current_cost < best_cost) {
                best_cost = current_cost;
                best_graph.adjacent = g.adjacent;
                best_graph.cost = best_cost;
            }
        }
    }
    return best_graph;
}
#endif // GRAPH_HELPER_H