#ifndef FERRY_MESSAGE // header and packet definition
#define FERRY_MESSAGE

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include <string>
#include <map>
#include <vector>
#include <unordered_map>

using namespace ns3;

struct Bundle {
    uint8_t hop;
    uint32_t id;
    Ipv4Address source;
    Ipv4Address destination;
    uint64_t creationTime; // in microseconds

    // some flag for managing bundle
    bool flag_waitingAck = false;
    EventId e_AckTimeout = EventId();
};

bool compareBundleTime(const Bundle& b1, const Bundle& b2) {
    return b1.creationTime < b2.creationTime;
}

class BundleHeader : public Header {
    private:
    uint8_t m_hop;
    uint32_t m_bundleId; // global bundle id = m_bundleID << 32 + m_source IP
    uint32_t m_sourceIp;
    uint32_t m_destIp;
    uint64_t m_creationTime; // creation time in micro second

    public:
    BundleHeader() : m_hop(0), m_bundleId(0), m_sourceIp(0), m_destIp(0), m_creationTime(0) {}

    void SetHop(uint8_t hop) { m_hop = hop; }
    uint8_t GetHop() const { return m_hop; }

    void SetBundleId(uint32_t id) { m_bundleId = id; }
    uint32_t GetBundleId() const { return m_bundleId; }

    void SetSourceIp(Ipv4Address ip) { m_sourceIp = ip.Get(); }
    Ipv4Address GetSourceIp() const { return Ipv4Address(m_sourceIp); }

    void SetDestIp(Ipv4Address ip) { m_destIp = ip.Get(); }
    Ipv4Address GetDestIp() const { return Ipv4Address(m_destIp); }

    void SetCreationTime(uint64_t time) { m_creationTime = time; }
    uint64_t GetCreationTime() const { return m_creationTime; }

    Bundle toBundle() const { // call when you want to decode header to bundle, note that hop is automatically incremented
        Bundle bundle;
        bundle.creationTime = m_creationTime;
        bundle.destination = Ipv4Address(m_destIp);
        bundle.hop = m_hop + 1;
        bundle.id = m_bundleId;
        bundle.source = Ipv4Address(m_sourceIp);
        return bundle;
    }

    static BundleHeader fromBundle(const Bundle& bundle) {
        BundleHeader header;
        header.SetHop(bundle.hop);
        header.SetBundleId(bundle.id);
        header.SetSourceIp(bundle.source);
        header.SetDestIp(bundle.destination);
        header.SetCreationTime(bundle.creationTime);
        return header;
    }

    static TypeId GetTypeId(void)
    {
        static TypeId tid = TypeId("ns3::BundleHeader")
            .SetParent<Header>()
            .AddConstructor<BundleHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Source=";
        os << (m_sourceIp >> 24) << ".";
        os << ((m_sourceIp >> 16) & 0xFF) << ".";
        os << ((m_sourceIp >> 8) & 0xFF) << ".";
        os << (m_sourceIp & 0xFF);
        os << " Bid=" << m_bundleId;
    }

    virtual uint32_t GetSerializedSize(void) const
    {
        return 1 + 4 + 4 + 4 + 8;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteU8(m_hop);
        start.WriteHtonU32(m_bundleId);
        start.WriteHtonU32(m_sourceIp);
        start.WriteHtonU32(m_destIp);
        start.WriteHtonU64(m_creationTime);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_hop = start.ReadU8();
        m_bundleId = start.ReadNtohU32();
        m_sourceIp = start.ReadNtohU32();
        m_destIp = start.ReadNtohU32();
        m_creationTime = start.ReadNtohU64();
        return GetSerializedSize();
    }
};

class BundleAckHeader : public Header {
    private:
    uint32_t m_bundleId; // global bundle id = m_bundleID << 32 + m_source IP
    uint32_t m_sourceIp;
    uint32_t m_destIp;

    public:
    BundleAckHeader() : m_bundleId(0), m_sourceIp(0), m_destIp(0) {}
    void SetBundleId(uint32_t id) { m_bundleId = id; }
    uint32_t GetBundleId() const { return m_bundleId; }

    void SetSourceIp(Ipv4Address ip) { m_sourceIp = ip.Get(); }
    Ipv4Address GetSourceIp() const { return Ipv4Address(m_sourceIp); }

    void SetDestIp(Ipv4Address ip) { m_destIp = ip.Get(); }
    Ipv4Address GetDestIp() const { return Ipv4Address(m_destIp); }

    static TypeId GetTypeId(void)
    {
        static TypeId tid = TypeId("ns3::BundleAckHeader")
            .SetParent<Header>()
            .AddConstructor<BundleAckHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "ACK Source=";
        os << (m_sourceIp >> 24) << ".";
        os << ((m_sourceIp >> 16) & 0xFF) << ".";
        os << ((m_sourceIp >> 8) & 0xFF) << ".";
        os << (m_sourceIp & 0xFF);
        os << " Bid=" << m_bundleId;
    }

    virtual uint32_t GetSerializedSize(void) const
    {
        return 4 + 4 + 4;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteHtonU32(m_bundleId);
        start.WriteHtonU32(m_sourceIp);
        start.WriteHtonU32(m_destIp);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_bundleId = start.ReadNtohU32();
        m_sourceIp = start.ReadNtohU32();
        m_destIp = start.ReadNtohU32();
        return GetSerializedSize();
    }
};

class MessageTypeHeader : public Header {
    private:
    uint8_t m_type;
    uint32_t m_nodeIp;

    public:
    enum MessageType {
        FERRY_BEACON = 1, // ferry broadcast on interval
        FERRY_HELLO = 2, // ferry unicast when receive beacon
        FERRY_ACCEPT_TRANSFER = 3, // ferry send to ferry node to tell it to send bundle

        GROUND_HELLO = 11, // ground node send when beaconed by ferry

        BUNDLE = 21, // bundle packet
        BUNDLE_ACK = 22, // bundle ack
        BUNDLE_ACK_FAT = 23, // bundle ack and ferry accept transfer, combined
    };

    MessageTypeHeader() : m_type(FERRY_HELLO), m_nodeIp(0) {}

    void SetType(uint8_t type) { m_type = type; }
    uint8_t GetType() const { return m_type; }

    void SetNodeIP(Ipv4Address ip) { m_nodeIp = ip.Get(); }
    Ipv4Address GetNodeIP() const { return Ipv4Address(m_nodeIp); }

    std::string GetMetaName() const {
        if (m_type == FERRY_BEACON) return "FERRY_BEACON";
        if (m_type == FERRY_HELLO) return "FERRY_HELLO";
        if (m_type == FERRY_ACCEPT_TRANSFER) return "FERRY_ACCEPT_TRANSFER";
        if (m_type == GROUND_HELLO) return "GROUND_HELLO";
        if (m_type == BUNDLE) return "BUNDLE";
        if (m_type == BUNDLE_ACK) return "BUNDLE_ACK";
        if (m_type == BUNDLE_ACK_FAT) return "BUNDLE_ACK_&_ACCEPT_TRANSFER";
        return "UNKNOWN";
    }

    static TypeId GetTypeId(void)
    {
        static TypeId tid = TypeId("ns3::MessageTypeHeader")
            .SetParent<Header>()
            .AddConstructor<MessageTypeHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Type=" << (int)m_type << " Source=" << m_nodeIp;
    }

    virtual uint32_t GetSerializedSize(void) const
    {
        return 1 + 4;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteU8(m_type);
        start.WriteHtonU32(m_nodeIp);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_type = start.ReadU8();
        m_nodeIp = start.ReadNtohU32();
        return GetSerializedSize();
    }
};

class FerryRouteHeader : public Header {
    private:
    uint8_t m_group;
    uint8_t m_mode;
    uint8_t m_count;
    std::vector<uint32_t> m_waypoints;
    std::vector<uint64_t> m_expectedArrival;

    public:
    FerryRouteHeader() : m_group(0), m_mode(0), m_count(0), m_waypoints(), m_expectedArrival() {}
    void SetGroup(uint8_t group) { m_group = group; }
    uint8_t GetGroup() const { return m_group; }

    void SetMode(uint8_t mode) { m_mode = mode; }
    uint8_t GetMode() const { return m_mode; }

    void SetCount(uint8_t count) { m_count = count; }
    uint8_t GetCount() const { return m_count; }

    void SetWaypoints(std::vector<uint32_t> waypoints) { m_waypoints = waypoints; }
    std::vector<uint32_t> GetWaypoints() const { return m_waypoints; }

    void SetExpectedArrival(std::vector<uint64_t> expectedArrival) { m_expectedArrival = expectedArrival; }
    std::vector<uint64_t> GetExpectedArrival() const { return m_expectedArrival; }

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::FerryRouteHeader")
            .SetParent<Header>()
            .AddConstructor<FerryRouteHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Group=" << (int)m_group;
    }

    virtual uint32_t GetSerializedSize(void) const
    {
        return 1 + 1 + 1 + (4 + 8) * m_count;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteU8(m_group);
        start.WriteU8(m_mode);
        start.WriteU8(m_count);
        for (uint8_t i = 0; i < m_count; i++) {
            start.WriteHtonU32(m_waypoints[i]);
            start.WriteHtonU64(m_expectedArrival[i]);
        }
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_group = start.ReadU8();
        m_mode = start.ReadU8();
        m_count = start.ReadU8();
        m_waypoints.resize(m_count);
        m_expectedArrival.resize(m_count);

        for (uint8_t i = 0; i < m_count; i++) {
            m_waypoints[i] = start.ReadNtohU32();
            m_expectedArrival[i] = start.ReadNtohU64();
        }
        return GetSerializedSize();
    }
};

class VisitTimeHeader : public Header {
    private:
    uint32_t count;
    std::vector<uint32_t> nodeIps;
    std::vector<uint64_t> lastVisitTimes;

    public:
    VisitTimeHeader() : count(0), nodeIps(), lastVisitTimes() {}

    void FromMap(std::unordered_map<uint32_t, uint64_t> map) {
        count = map.size();
        nodeIps.resize(count);
        lastVisitTimes.resize(count);
        uint32_t i = 0;
        for (auto it = map.begin(); it != map.end(); it++) {
            nodeIps[i] = it->first;
            lastVisitTimes[i] = it->second;
            i++;
        }
    }
    std::unordered_map<uint32_t, uint64_t> ToMap() {
        std::unordered_map<uint32_t, uint64_t> map;
        for (uint32_t i = 0; i < count; i++) {
            map[nodeIps[i]] = lastVisitTimes[i];
        }
        return map;
    }

    void SetCount(uint32_t count) { this->count = count; }
    uint32_t GetCount() const { return count; }

    void SetNodeIps(std::vector<uint32_t> nodeIps) { this->nodeIps = nodeIps; }
    std::vector<uint32_t> GetNodeIps() const { return nodeIps; }

    void SetLastVisitTimes(std::vector<uint64_t> lastVisitTimes) { this->lastVisitTimes = lastVisitTimes; }
    std::vector<uint64_t> GetLastVisitTimes() const { return lastVisitTimes; }

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VisitTimeHeader")
            .SetParent<Header>()
            .AddConstructor<VisitTimeHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Count=" << count;
    }

    virtual uint32_t GetSerializedSize(void) const {
        return 4 + (4 + 8) * count;
    }

    virtual void Serialize(Buffer::Iterator start) const {
        start.WriteHtonU32(count);
        for (uint32_t i = 0; i < count; i++) {
            start.WriteHtonU32(nodeIps[i]);
            start.WriteHtonU64(lastVisitTimes[i]);
        }
    }

    virtual uint32_t Deserialize(Buffer::Iterator start) {
        count = start.ReadNtohU32();
        nodeIps.resize(count);
        lastVisitTimes.resize(count);
        for (uint32_t i = 0; i < count; i++) {
            nodeIps[i] = start.ReadNtohU32();
            lastVisitTimes[i] = start.ReadNtohU64();
        }
        return GetSerializedSize();
    }

};

class BufferStateHeader : public Header {
    private:
    uint32_t capacity;
    uint32_t count;
    std::vector<uint32_t> nodeIps;
    std::vector<uint32_t> messageCount;

    public:
    BufferStateHeader() : capacity(0), count(0), nodeIps(), messageCount() {}

    void SetCapacity(uint32_t capacity) { this->capacity = capacity; }
    uint32_t GetCapacity() const { return capacity; }

    void SetCount(uint32_t count) { this->count = count; }
    uint32_t GetCount() const { return count; }

    void SetNodeIps(std::vector<uint32_t> nodeIps) { this->nodeIps = nodeIps; }
    std::vector<uint32_t> GetNodeIps() const { return nodeIps; }

    void SetMessageCount(std::vector<uint32_t> messageCount) { this->messageCount = messageCount; }
    std::vector<uint32_t> GetMessageCount() const { return messageCount; }

    void FromBuffer(std::vector<Bundle> buffer) {
        std::map<uint32_t, uint32_t> countMap;
        for (auto bundle : buffer) {
            if (countMap.find(bundle.destination.Get()) == countMap.end()) {
                countMap[bundle.destination.Get()] = 1;
            }
            else {
                countMap[bundle.destination.Get()]++;
            }
        }
        count = countMap.size();
        nodeIps.resize(count);
        messageCount.resize(count);

        uint32_t i = 0;
        for (auto it = countMap.begin(); it != countMap.end(); it++) {
            nodeIps[i] = it->first;
            messageCount[i] = it->second;
            i++;
        }
    }

    std::map<uint32_t, uint32_t> ToCountMap() const {
        std::map<uint32_t, uint32_t> countMap;
        for (uint32_t i = 0; i < count; i++) {
            countMap[nodeIps[i]] = messageCount[i];
        }
        return countMap;
    }

    bool IsFull() const {
        uint32_t totalMessages = 0;
        for (uint32_t i = 0; i < count; i++) {
            totalMessages += messageCount[i];
        }
        return totalMessages >= capacity;
    }

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::BufferStateHeader")
            .SetParent<Header>()
            .AddConstructor<BufferStateHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }

    virtual void Print(std::ostream& os) const
    {
        os << "Count=" << count;
    }

    virtual uint32_t GetSerializedSize(void) const {
        return 4 + 4 + (4 + 4) * count;
    }

    virtual void Serialize(Buffer::Iterator start) const {
        start.WriteHtonU32(capacity);
        start.WriteHtonU32(count);
        for (uint32_t i = 0; i < count; i++) {
            start.WriteHtonU32(nodeIps[i]);
            start.WriteHtonU32(messageCount[i]);
        }
    }

    virtual uint32_t Deserialize(Buffer::Iterator start) {
        capacity = start.ReadNtohU32();
        count = start.ReadNtohU32();
        nodeIps.resize(count);
        messageCount.resize(count);
        for (uint32_t i = 0; i < count; i++) {
            nodeIps[i] = start.ReadNtohU32();
            messageCount[i] = start.ReadNtohU32();
        }
        return GetSerializedSize();
    }

};
#endif // FERRY_MESSAGE