#ifndef NANOKOTON_IP_HPP
#define NANOKOTON_IP_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/net/ethernet.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/hashmap.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/lib/queue.hpp>

namespace nk::net {
    struct PACKED IPv4Header {
        u8 version_ihl;
        u8 dscp_ecn;
        u16 total_length;
        u16 identification;
        u16 flags_fragment_offset;
        u8 time_to_live;
        u8 protocol;
        u16 header_checksum;
        u32 source_address;
        u32 destination_address;
        
        u8 version() const { return version_ihl >> 4; }
        u8 ihl() const { return version_ihl & 0x0F; }
        u16 header_length() const { return ihl() * 4; }
        
        u8 dscp() const { return dscp_ecn >> 2; }
        u8 ecn() const { return dscp_ecn & 0x03; }
        
        u16 fragment_offset() const { return flags_fragment_offset & 0x1FFF; }
        bool dont_fragment() const { return flags_fragment_offset & 0x4000; }
        bool more_fragments() const { return flags_fragment_offset & 0x2000; }
    };

    enum class IPProtocol : u8 {
        ICMP = 1,
        TCP = 6,
        UDP = 17,
        ICMPv6 = 58
    };

    struct IPAddress {
        union {
            u32 ipv4;
            u8 ipv4_bytes[4];
            u64 ipv6[2];
            u8 ipv6_bytes[16];
        };
        
        bool is_ipv4;
        
        IPAddress() : ipv4(0), is_ipv4(true) {}
        IPAddress(u32 addr) : ipv4(addr), is_ipv4(true) {}
        IPAddress(u8 a, u8 b, u8 c, u8 d) : is_ipv4(true) {
            ipv4_bytes[0] = a;
            ipv4_bytes[1] = b;
            ipv4_bytes[2] = c;
            ipv4_bytes[3] = d;
        }
        
        static IPAddress broadcast() { return IPAddress(255, 255, 255, 255); }
        static IPAddress any() { return IPAddress(0, 0, 0, 0); }
        static IPAddress localhost() { return IPAddress(127, 0, 0, 1); }
        
        bool operator==(const IPAddress& other) const {
            if (is_ipv4 != other.is_ipv4) return false;
            if (is_ipv4) return ipv4 == other.ipv4;
            return ipv6[0] == other.ipv6[0] && ipv6[1] == other.ipv6[1];
        }
        
        bool operator!=(const IPAddress& other) const {
            return !(*this == other);
        }
    };

    struct IPPacket {
        IPAddress source;
        IPAddress destination;
        IPProtocol protocol;
        u16 identification;
        u8 time_to_live;
        Vector<u8> data;
        bool is_fragment;
        u16 fragment_offset;
        bool more_fragments;
    };

    class IPLayer {
    private:
        struct FragmentBuffer {
            u16 identification;
            IPAddress source;
            IPAddress destination;
            IPProtocol protocol;
            HashMap<u16, Vector<u8>> fragments;
            u64 last_accessed;
            u32 total_length;
            u32 received_length;
            bool is_complete;
        };
        
        struct RouteEntry {
            IPAddress network;
            IPAddress gateway;
            IPAddress netmask;
            u32 interface_index;
            u32 metric;
        };
        
        struct Interface {
            u32 index;
            IPAddress address;
            IPAddress netmask;
            IPAddress gateway;
            u8 mac_address[6];
            bool is_up;
            u32 mtu;
            EthernetDevice* device;
        };
        
        HashMap<u64, FragmentBuffer> fragment_buffers_;
        Vector<RouteEntry> routing_table_;
        Vector<Interface> interfaces_;
        Mutex lock_;
        
        u16 identification_counter_;
        u32 default_interface_index_;
        
        using PacketCallback = void (*)(const IPPacket& packet, void* user_data);
        
        struct ProtocolHandler {
            IPProtocol protocol;
            PacketCallback callback;
            void* user_data;
        };
        
        Vector<ProtocolHandler> protocol_handlers_;
        Mutex callback_lock_;
        
        u16 calculate_checksum(const u8* data, usize length);
        bool validate_packet(const u8* buffer, usize size);
        void process_fragment(const IPv4Header* header, const u8* data, usize size);
        void reassemble_packets();
        void cleanup_old_fragments();
        
        bool find_route(const IPAddress& destination, RouteEntry* route);
        u32 find_interface_for_address(const IPAddress& address);
        
        void send_packet_to_interface(u32 interface_index, const IPAddress& destination,
                                     IPProtocol protocol, const u8* data, usize size);
        
    public:
        IPLayer();
        ~IPLayer();
        
        bool init();
        
        bool add_interface(u32 device_index, const IPAddress& address,
                          const IPAddress& netmask, const IPAddress& gateway);
        bool remove_interface(u32 interface_index);
        
        bool set_interface_address(u32 interface_index, const IPAddress& address);
        bool set_interface_netmask(u32 interface_index, const IPAddress& netmask);
        bool set_interface_gateway(u32 interface_index, const IPAddress& gateway);
        
        const Interface* get_interface(u32 index) const;
        usize get_interface_count() const { return interfaces_.size(); }
        
        bool add_route(const IPAddress& network, const IPAddress& netmask,
                      const IPAddress& gateway, u32 interface_index, u32 metric = 0);
        bool remove_route(const IPAddress& network, const IPAddress& netmask);
        
        bool send_packet(const IPAddress& destination, IPProtocol protocol,
                        const u8* data, usize size);
        
        bool register_protocol_handler(IPProtocol protocol, PacketCallback callback,
                                      void* user_data);
        bool unregister_protocol_handler(IPProtocol protocol, PacketCallback callback);
        
        void process_packet(const u8* source_mac, const u8* destination_mac,
                           const u8* buffer, usize size);
        
        void poll();
        
        void dump_interfaces() const;
        void dump_routing_table() const;
        void dump_fragment_buffers() const;
        
        static IPLayer& instance();
    };
}
#endif
