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


// khoảng cách theo số bước nhảy, BFS 
std::vector<std::vector<uint32_t>> GetGraphDistanceMatrix(const Graph& g) {
    std::vector<std::vector<uint32_t>> dists(g.n, std::vector<uint32_t>(g.n, 999));
    for (uint32_t i = 0; i < g.n; ++i) {
        dists[i][i] = 0;
        std::vector<uint32_t> q;
        q.push_back(i);
        uint32_t head = 0;
        while (head < q.size()) {
            uint32_t u = q[head++];
            for (uint32_t v : g.adjacent[u]) {
                if (dists[i][v] == 999) {
                    dists[i][v] = dists[i][u] + 1;
                    q.push_back(v);
                }
            }
        }
    }
    return dists;
}

void bronKerbosch(std::set<uint32_t> R, std::set<uint32_t> P, std::set<uint32_t> X,
                  const std::vector<std::vector<bool>>& graphMtrx,
                  std::vector<std::set<uint32_t>>& maximalCliques) {
    if (P.empty() && X.empty()) {
        if (R.size() >= 2) maximalCliques.push_back(R);
        return;
    }
    auto it = P.begin();
    while (it != P.end()) {
        uint32_t v = *it;
        std::set<uint32_t> newR = R; newR.insert(v);
        std::set<uint32_t> newP, newX;
        for (uint32_t neighbor = 0; neighbor < graphMtrx.size(); ++neighbor) {
            if (graphMtrx[v][neighbor]) {
                if (P.count(neighbor)) newP.insert(neighbor);
                if (X.count(neighbor)) newX.insert(neighbor);
            }
        }
        bronKerbosch(newR, newP, newX, graphMtrx, maximalCliques);
        P.erase(it++);
        X.insert(v);
    }
}

std::vector<std::vector<uint32_t>> findInitialHubs(const Graph& g) {
    auto path_dists = GetGraphDistanceMatrix(g);

    // ma trận kết nối bậc 2, 2 nút kết nối với nhau nếu khoảng cách của chúng trên đò thị g <= 2
    std::vector<std::vector<bool>> g2Mtrx(g.n, std::vector<bool>(g.n, false));
    std::set<std::pair<uint32_t, uint32_t>> edgesG2;

    for (uint32_t i = 0; i < g.n; ++i) {
        for (uint32_t j = i + 1; j < g.n; ++j) {
            if (path_dists[i][j] <= 2) {
                g2Mtrx[i][j] = g2Mtrx[j][i] = true;
                edgesG2.insert({ i, j });
            }
        }
    }

    std::vector<std::set<uint32_t>> cliques;
    std::set<uint32_t> P;
    for (uint32_t i = 0; i < g.n; ++i) P.insert(i);
    bronKerbosch({}, P, {}, g2Mtrx, cliques);

    // Tham lam phủ hết các cạnh G2
    std::vector<std::vector<uint32_t>> selectedHubs;
    while (!edgesG2.empty()) { // Khi nào còn cạnh chưa được phủ bởi 1 hub
        int bestIdx = -1;
        std::set<std::pair<uint32_t, uint32_t>> bestCovered;

        for (int k = 0; k < cliques.size(); ++k) {
            std::set<std::pair<uint32_t, uint32_t>> covered;
            std::vector<uint32_t> nodes(cliques[k].begin(), cliques[k].end());
            for (size_t i = 0; i < nodes.size(); ++i) {
                for (size_t j = i + 1; j < nodes.size(); ++j) {
                    std::pair<uint32_t, uint32_t> edge = { std::min(nodes[i], nodes[j]), std::max(nodes[i], nodes[j]) };
                    if (edgesG2.count(edge)) covered.insert(edge);
                }
            }
            if ((int)covered.size() > (int)bestCovered.size()) { // chọn clique phủ được nhiều cạnh còn lại nhất
                bestCovered = covered;
                bestIdx = k;
            }
        }

        if (bestIdx != -1) {
            selectedHubs.push_back(std::vector<uint32_t>(cliques[bestIdx].begin(), cliques[bestIdx].end()));
            for (auto const& e : bestCovered) edgesG2.erase(e); // loại bỏ các cạnh đã được phủ
        }
        else break;
    }
    return selectedHubs;
}

double getDiameter(const std::vector<uint32_t>& idx, const std::vector<point2D>& points) {
    double maxD = 0;
    for (size_t i = 0; i < idx.size(); ++i) {
        for (size_t j = i + 1; j < idx.size(); ++j) {
            maxD = std::max(maxD, dist(points[idx[i]], points[idx[j]]));
        }
    }
    return maxD;
}

std::vector<std::vector<uint32_t>> mergeHubsToK(std::vector<std::vector<uint32_t>> hubs, const Graph& g, uint32_t k) {
    while (hubs.size() > k) {
        int bestI = -1, bestJ = -1;
        double minDiameter = 1e18;

        for (size_t i = 0; i < hubs.size(); ++i) {
            for (size_t j = i + 1; j < hubs.size(); ++j) {
                // đường kính khi gộp 2 hubs làm 1
                double d = getDiameter(hubs[i], g.points);
                d = std::max(d, getDiameter(hubs[j], g.points));
                for (uint32_t a : hubs[i]) {
                    for (uint32_t b : hubs[j]) {
                        d = std::max(d, dist(g.points[a], g.points[b]));
                    }
                }
                if (d < minDiameter) {
                    minDiameter = d;
                    bestI = i; bestJ = j;
                }
            }
        }

        if (bestI != -1) {
            // Gộp bestJ vào bestI, sau đó xóa bestJ
            for (uint32_t node : hubs[bestJ]) {
                if (std::find(hubs[bestI].begin(), hubs[bestI].end(), node) == hubs[bestI].end())
                    hubs[bestI].push_back(node);
            }
            hubs.erase(hubs.begin() + bestJ);
        }
        else break;
    }
    return hubs;
}

std::vector<std::vector<uint32_t>> GraphBasedSoft2CliqueCluster(const Graph& g, uint32_t k) {
    std::vector<std::vector<uint32_t>> hubs = findInitialHubs(g);
    hubs = mergeHubsToK(hubs, g, k);
    return hubs;
}

Graph BuildGraphFromHubs(const std::vector<std::vector<uint32_t>>& hubs, uint32_t n, uint32_t firstHubIndex) {
    Graph g;
    g.n = n;
    g.points = {};
    g.adjacent.resize(n);
    for (auto hub : hubs) {
        g.adjacent[firstHubIndex] = hub;
        for (auto node : hub) {
            g.adjacent[node].push_back(firstHubIndex);
        }
        firstHubIndex++;
    }

    return g;
}

/**
 * Gabriel Graph
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
 * 1-CENTER (STAR GRAPH) with specified center index
 */
Graph BuildOneCenterGraph(const uint32_t n, const uint32_t c) {
    Graph g; g.n = n;
    g.points = {};
    g.adjacent.resize(n);

    for (uint32_t i = 0; i < n; ++i) {
        if (i != c) {
            g.adjacent[c].push_back(i);
            g.adjacent[i].push_back(c);
        }
    }
    return g;
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

Graph BuildFullyConnectedGraph(const uint32_t n) {
    Graph g; g.n = n;
    g.points = {};
    g.adjacent.resize(n);

    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t j = i + 1; j < n; ++j) {
            g.adjacent[i].push_back(j);
            g.adjacent[j].push_back(i);
        }
    }
    return g;
}
#endif // GRAPH_HELPER_H