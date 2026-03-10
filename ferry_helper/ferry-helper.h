#ifndef FERRY_HELPER_H
#define FERRY_HELPER_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include "ns3/core-module.h"

#include "datatypes.h"
#include "packet-helper.h"

std::vector<point2D> PoissonDisk_BridsonSample(uint32_t n, double r, double areaWidth) {
    std::vector<point2D> points;

    const int k = 30; // Số lần thử (rejection limit) cho mỗi điểm hoạt động, chuẩn là 30
    double cellSize = r / std::sqrt(2.0); // Kích thước ô lưới để đảm bảo mỗi ô chỉ chứa tối đa 1 điểm

    // Số lượng ô lưới theo chiều ngang/dọc
    int gridSize = std::ceil(areaWidth / cellSize);

    // Lưới lưu trữ chỉ số (index) của điểm trong vector 'points'. -1 nghĩa là ô trống.
    // Dùng vector 1 chiều để mô phỏng mảng 2 chiều: index = y * gridSize + x
    std::vector<int> grid(gridSize * gridSize, -1);

    // Danh sách "active list" chứa các index của điểm cần xem xét
    std::vector<int> activeList;

    // 2. Khởi tạo điểm đầu tiên ngẫu nhiên
    point2D p0 = { m_rand->GetValue(0, areaWidth) , m_rand->GetValue(0, areaWidth) };
    points.push_back(p0);

    int col0 = (int)(p0.x / cellSize);
    int row0 = (int)(p0.y / cellSize);
    grid[row0 * gridSize + col0] = 0; // Lưu index 0 vào lưới
    activeList.push_back(0);

    // 3. Vòng lặp chính
    while (!activeList.empty() && points.size() < n) {
        // Chọn ngẫu nhiên một điểm từ active list
        int randIdx = m_rand->GetInteger(0, activeList.size() - 1);
        int pointIdx = activeList[randIdx];
        point2D center = points[pointIdx];

        bool found = false;

        // Thử k lần để sinh điểm mới xung quanh điểm center
        for (int i = 0; i < k; ++i) {
            double angle = m_rand->GetValue(0, 2 * M_PI);
            double dist = m_rand->GetValue(0, r);

            double newX = center.x + std::cos(angle) * dist;
            double newY = center.y + std::sin(angle) * dist;

            // Kiểm tra biên (Boundary check)
            if (newX < 0 || newX >= areaWidth || newY < 0 || newY >= areaWidth) {
                continue;
            }

            // Kiểm tra lưới (Grid check)
            int col = (int)(newX / cellSize);
            int row = (int)(newY / cellSize);
            bool collision = false;

            // Kiểm tra các ô lân cận (trong khoảng 5x5 ô xung quanh)
            // Vì cell = r/sqrt(2), nên kiểm tra +-2 ô là đủ bao phủ bán kính r
            for (int rOffset = -2; rOffset <= 2; ++rOffset) {
                for (int cOffset = -2; cOffset <= 2; ++cOffset) {
                    int checkRow = row + rOffset;
                    int checkCol = col + cOffset;

                    // Bỏ qua nếu ra ngoài lưới
                    if (checkRow < 0 || checkRow >= gridSize || checkCol < 0 || checkCol >= gridSize)
                        continue;

                    int neighborIdx = grid[checkRow * gridSize + checkCol];
                    if (neighborIdx != -1) {
                        double d2 = distSq({ newX, newY }, points[neighborIdx]);
                        if (d2 < r * r) {
                            collision = true;
                            goto end_check; // Thoát khỏi 2 vòng lặp lồng nhau
                        }
                    }
                }
            }

        end_check:;

        // Nếu vị trí hợp lệ
            if (!collision) {
                point2D newP = { newX, newY };
                points.push_back(newP);

                // Cập nhật Grid và Active List
                int newIdx = points.size() - 1;
                grid[row * gridSize + col] = newIdx;
                activeList.push_back(newIdx);

                found = true;

                // Nếu đã đủ n điểm thì dừng ngay lập tức
                if (points.size() >= n) return points;

                break; // Dừng thử k lần, quay lại vòng lặp chính
            }
        }

        // Nếu sau k lần thử không tìm được điểm nào, loại điểm hiện tại khỏi active list
        if (!found) {
            activeList.erase(activeList.begin() + randIdx);
        }
    }
    NS_ASSERT(points.size() == n);

    return points;
}

std::vector<point2D> PoissonDisk_RandomSample(uint32_t n, double r, double areaWidth) {
    std::vector<point2D> points;
    double rSq = r * r; // So sánh với r^2 để tránh dùng hàm sqrt()

    // Cài đặt bộ sinh số ngẫu nhiên chuẩn

    int maxAttemptsTotal = n * 1000; // Giới hạn số lần thử để tránh treo máy nếu không gian quá chật
    int attempts = 0;

    while (points.size() < n && attempts < maxAttemptsTotal) {
        // 1. Sinh ngẫu nhiên toàn cục (bất kỳ đâu trên bản đồ)
        point2D candidate = { m_rand->GetValue(0, areaWidth), m_rand->GetValue(0, areaWidth) };

        bool collision = false;

        // 2. Kiểm tra khoảng cách với TẤT CẢ các điểm đã có
        // Với n=50, vòng lặp này chạy rất nhanh. 
        // Nếu n lớn (ví dụ > 5000), cách này sẽ chậm và cần tối ưu bằng Grid.
        for (const auto& p : points) {
            if (distSq(candidate, p) < rSq) {
                collision = true;
                break;
            }
        }

        // 3. Nếu không va chạm, thêm vào danh sách
        if (!collision) {
            points.push_back(candidate);
        }

        attempts++;
    }

    if (points.size() < n) {
        NS_LOG_UNCOND("Random Sample Failed, trying to Bridson Sample");
        return PoissonDisk_BridsonSample(n, r, areaWidth);
    }
    NS_ASSERT(points.size() == n);

    return points;
}

uint64_t CalExpectedArrival(const uint32_t node, const std::vector<uint32_t> nodeIps, const std::vector<uint64_t> visitTime) {
    for (uint32_t i = 0; i < nodeIps.size(); i++) {
        if (nodeIps[i] == node) {
            return visitTime[i];
        }
    }
    return 0;
}

double CalRouteDistance(const std::vector<point2D>& points, const std::vector<uint32_t>& route, int startIndex, int endIndex, int direction) {
    double distance = 0;
    while (startIndex != endIndex) {
        // NS_LOG_UNCOND(startIndex);
        int nextIndex = (startIndex + direction + route.size()) % route.size();
        distance += dist(points[route[startIndex]], points[route[nextIndex]]);
        startIndex = nextIndex;
    }
    return distance;
}
#endif // FERRY_HELPER_H