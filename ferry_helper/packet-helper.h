#ifndef FERRY_MESSAGE // header and packet definition
#define FERRY_MESSAGE

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include <string>

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

class BundleHeader : public Header
{
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

class BundleAckHeader : public Header
{
    private:
    uint32_t m_bundleId; // global bundle id = m_bundleID << 32 + m_source IP
    uint32_t m_sourceIp;
    uint32_t m_destIp;
    bool m_allowSendNext; // allow node to send next bundle, this is just for node to signal that its buffer is full

    public:
    BundleAckHeader() : m_bundleId(0), m_sourceIp(0), m_destIp(0), m_allowSendNext(false) {}
    void SetBundleId(uint32_t id) { m_bundleId = id; }
    uint32_t GetBundleId() const { return m_bundleId; }

    void SetSourceIp(Ipv4Address ip) { m_sourceIp = ip.Get(); }
    Ipv4Address GetSourceIp() const { return Ipv4Address(m_sourceIp); }

    void SetDestIp(Ipv4Address ip) { m_destIp = ip.Get(); }
    Ipv4Address GetDestIp() const { return Ipv4Address(m_destIp); }

    void SetAllowSendNext(bool allow) { m_allowSendNext = allow; }
    bool GetAllowSendNext() const { return m_allowSendNext; }

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
        return 4 + 4 + 4 + 1;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteHtonU32(m_bundleId);
        start.WriteHtonU32(m_sourceIp);
        start.WriteHtonU32(m_destIp);
        uint8_t flags;
        flags = m_allowSendNext ? 1 : 0;
        start.WriteU8(flags);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_bundleId = start.ReadNtohU32();
        m_sourceIp = start.ReadNtohU32();
        m_destIp = start.ReadNtohU32();
        uint8_t flags = start.ReadU8();
        m_allowSendNext = flags == 1;
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
        FERRY_ACCEPT_TRANSFER = 3, // ferry send after tranfering all of the bundle it need to send to the ground node 

        GROUND_HELLO = 11, // ground node send when beaconed by ferry

        BUNDLE = 21, // bundle packet, a ground node sent this
        BUNDLE_ACK = 22, // bundle ack, a ground node sent this
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
    uint8_t m_count;
    std::vector<uint32_t> m_waypoints;
    std::vector<uint64_t> m_expectedArrival;

    public:
    FerryRouteHeader() : m_group(0), m_count(0), m_waypoints() {}
    void SetGroup(uint8_t group) { m_group = group; }
    uint8_t GetGroup() const { return m_group; }

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
        return 1 + 1 + (4 + 8) * m_count;
    }

    virtual void Serialize(Buffer::Iterator start) const
    {
        start.WriteU8(m_group);
        start.WriteU8(m_count);
        for (uint8_t i = 0; i < m_count; i++) {
            start.WriteHtonU32(m_waypoints[i]);
            start.WriteHtonU64(m_expectedArrival[i]);
        }
    }

    virtual uint32_t Deserialize(Buffer::Iterator start)
    {
        m_group = start.ReadU8();
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


#endif // FERRY_MESSAGE