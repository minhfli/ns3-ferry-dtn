#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include <vector>
#include <cmath>

using namespace ns3;

// --- CLASS ĐIỀU KHIỂN UAV (Nằm ngay trong file scratch) ---
class UavController : public Object {
    public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("UavController")
            .SetParent<Object>()
            .SetGroupName("Tutorial")
            .AddConstructor<UavController>();
        return tid;
    }

    UavController() : m_speed(20.0), m_isRunning(false) {}

    // Hàm khởi tạo: Gắn với Node và MobilityModel của Node đó
    void Setup(Ptr<Node> node, double speed, std::vector<Vector> waypoints) {
        m_node = node;
        m_speed = speed;
        m_waypoints = waypoints;
        m_currentWpIndex = 0;

        // Lấy pointer đến MobilityModel đã được cài sẵn trong node
        m_mobility = node->GetObject<ConstantVelocityMobilityModel>();

        if (!m_mobility) {
            NS_FATAL_ERROR("Node chưa được cài ConstantVelocityMobilityModel!");
        }
    }

    void Start() {
        if (m_waypoints.empty()) return;
        m_isRunning = true;

        // Đặt vị trí ban đầu là waypoint đầu tiên
        m_mobility->SetPosition(m_waypoints[0]);

        // Bắt đầu đi đến waypoint tiếp theo
        ScheduleNextMove();
    }

    private:
    void ScheduleNextMove() {
        if (!m_isRunning) return;

        // 1. Xác định điểm đến (Logic chọn đường của bạn nằm ở đây)
        // Ví dụ: Đi vòng tròn (Round Robin)
        m_currentWpIndex = (m_currentWpIndex + 1) % m_waypoints.size();
        Vector destination = m_waypoints[m_currentWpIndex];

        // 2. Lấy vị trí hiện tại
        Vector currentPos = m_mobility->GetPosition();

        // 3. Tính toán Vector vận tốc
        double dx = destination.x - currentPos.x;
        double dy = destination.y - currentPos.y;
        double dz = destination.z - currentPos.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < 1.0) {
            // Đã rất gần đích -> Bỏ qua, đi tiếp điểm sau ngay lập tức
            Simulator::ScheduleNow(&UavController::ScheduleNextMove, this);
            return;
        }

        double timeToTravel = distance / m_speed;
        Vector velocity(dx / timeToTravel, dy / timeToTravel, dz / timeToTravel);

        // 4. RA LỆNH CHO MOBILITY MODEL (Đây là mấu chốt)
        m_mobility->SetVelocity(velocity);

        NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds()
                       << "s | Node " << m_node->GetId()
                       << " bắt đầu bay tới " << destination
                       << " trong " << timeToTravel << "s");

        // 5. Hẹn giờ khi đến nơi thì gọi lại hàm này
        Simulator::Schedule(Seconds(timeToTravel), &UavController::ScheduleNextMove, this);
    }

    Ptr<Node> m_node;
    Ptr<ConstantVelocityMobilityModel> m_mobility;
    double m_speed;
    std::vector<Vector> m_waypoints;
    size_t m_currentWpIndex;
    bool m_isRunning;
};

// --- HÀM MAIN ---
int main(int argc, char* argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);
    // 1. Tạo Node
    NodeContainer nodes;
    nodes.Create(1);

    // 2. CÀI ĐẶT MOBILITY THỦ CÔNG (KHÔNG DÙNG HELPER)
    // Bản chất MobilityHelper chỉ làm việc này giúp bạn thôi:
    Ptr<ConstantVelocityMobilityModel> mob = CreateObject<ConstantVelocityMobilityModel>();
    nodes.Get(0)->AggregateObject(mob); // Gắn Model vào Node

    // 3. Cài đặt Controller của chúng ta
    std::vector<Vector> myPath;
    myPath.push_back(Vector(0, 0, 50));
    myPath.push_back(Vector(100, 0, 50));
    myPath.push_back(Vector(100, 100, 50));
    myPath.push_back(Vector(0, 100, 50));

    Ptr<UavController> controller = CreateObject<UavController>();
    controller->Setup(nodes.Get(0), 25.0, myPath); // Tốc độ 25 m/s

    // 4. Kích hoạt
    Simulator::Schedule(Seconds(1.0), &UavController::Start, controller);

    // Chạy mô phỏng
    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}