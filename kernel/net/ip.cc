#include <nanokoton/net/ip.hpp>
#include <nanokoton/net/ethernet.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/algorithm.hpp>
#include <nanokoton/arch/cpu.hpp>

namespace nk::net {
    IPLayer::IPLayer() 
        : identification_counter_(0),
          default_interface_index_(0) {
        debug::log(debug::LogLevel::Info, "IP", "IP Layer created");
    }

    IPLayer::~IPLayer() {
        ScopedLock lock(lock_);
        
        for (Interface& iface : interfaces_) {
            delete iface.device;
        }
        interfaces_.clear();
        
        fragment_buffers_.clear();
        routing_table_.clear();
        protocol_handlers_.clear();
    }

    bool IPLayer::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "IP", "Initializing IP Layer");
        
        EthernetManager& eth_mgr = EthernetManager::instance();
        eth_mgr.register_callback(static_cast<u16>(EthernetHeader::Type::IPv4),
                                 [](const u8* source, const u8* destination,
                                    u16 ether_type, const u8* data, usize size,
                                    void* user_data) {
            IPLayer* ip = static_cast<IPLayer*>(user_data);
            ip->process_packet(source, destination, data, size);
        }, this);
        
        debug::log(debug::LogLevel::Info, "IP", "IP Layer initialized");
        return true;
    }

    u16 IPLayer::calculate_checksum(const u8* data, usize length) {
        u32 sum = 0;
        const u16* ptr = reinterpret_cast<const u16*>(data);
        
        while (length > 1) {
            sum += *ptr++;
            length -= 2;
        }
        
        if (length > 0) {
            sum += *reinterpret_cast<const u8*>(ptr);
        }
        
        while (sum >> 16) {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        
        return static_cast<u16>(~sum);
    }

    bool IPLayer::validate_packet(const u8* buffer, usize size) {
        if (size < sizeof(IPv4Header)) {
            return false;
        }
        
        const IPv4Header* header = reinterpret_cast<const IPv4Header*>(buffer);
        
        if (header->version() != 4) {
            return false;
        }
        
        if (header->ihl() < 5) {
            return false;
        }
        
        u16 header_length = header->header_length();
        if (header_length > size) {
            return false;
        }
        
        u16 checksum = header->checksum;
        const_cast<IPv4Header*>(header)->checksum = 0;
        u16 calculated = calculate_checksum(buffer, header_length);
        const_cast<IPv4Header*>(header)->checksum = checksum;
        
        if (checksum != calculated) {
            return false;
        }
        
        u16 total_length = swap_endian_16(header->total_length);
        if (total_length > size || total_length < header_length) {
            return false;
        }
        
        return true;
    }

    void IPLayer::process_fragment(const IPv4Header* header, const u8* data, usize size) {
        u16 fragment_offset = header->fragment_offset();
        u16 identification = swap_endian_16(header->identification);
        u32 source_ip = header->source_address;
        u32 dest_ip = header->destination_address;
        u8 protocol = header->protocol;
        
        u64 key = (static_cast<u64>(source_ip) << 32) | dest_ip;
        key = (key << 16) | identification;
        key = (key << 8) | protocol;
        
        FragmentBuffer* buffer = nullptr;
        auto it = fragment_buffers_.find(key);
        
        if (it == fragment_buffers_.end()) {
            buffer = new FragmentBuffer();
            buffer->identification = identification;
            buffer->source.ipv4 = source_ip;
            buffer->destination.ipv4 = dest_ip;
            buffer->protocol = static_cast<IPProtocol>(protocol);
            buffer->last_accessed = arch::CPU::read_tsc();
            buffer->total_length = 0;
            buffer->received_length = 0;
            buffer->is_complete = false;
            
            fragment_buffers_[key] = buffer;
        } else {
            buffer = it->second;
        }
        
        buffer->last_accessed = arch::CPU::read_tsc();
        
        u16 offset = fragment_offset * 8;
        bool more_fragments = header->more_fragments();
        
        Vector<u8> fragment_data(size - header->header_length());
        memcpy(fragment_data.data(), data + header->header_length(), fragment_data.size());
        
        buffer->fragments[offset] = fragment_data;
        buffer->received_length += fragment_data.size();
        
        if (!more_fragments) {
            u16 total_len = swap_endian_16(header->total_length);
            buffer->total_length = offset + fragment_data.size();
        }
        
        if (buffer->total_length > 0) {
            u32 expected_len = buffer->total_length;
            u32 actual_len = 0;
            u32 last_offset = 0;
            bool complete = true;
            
            for (const auto& frag : buffer->fragments) {
                if (frag.first != last_offset) {
                    complete = false;
                    break;
                }
                actual_len += frag.second.size();
                last_offset += frag.second.size();
            }
            
            if (complete && actual_len == expected_len) {
                buffer->is_complete = true;
                
                Vector<u8> reassembled(expected_len);
                u32 pos = 0;
                
                for (const auto& frag : buffer->fragments) {
                    memcpy(reassembled.data() + pos, frag.second.data(), frag.second.size());
                    pos += frag.second.size();
                }
                
                IPPacket packet;
                packet.source.ipv4 = buffer->source.ipv4;
                packet.destination.ipv4 = buffer->destination.ipv4;
                packet.protocol = buffer->protocol;
                packet.identification = buffer->identification;
                packet.data = reassembled;
                packet.is_fragment = false;
                packet.fragment_offset = 0;
                packet.more_fragments = false;
                
                ScopedLock callback_lock(callback_lock_);
                for (const ProtocolHandler& handler : protocol_handlers_) {
                    if (handler.protocol == packet.protocol) {
                        handler.callback(packet, handler.user_data);
                    }
                }
                
                fragment_buffers_.erase(key);
                delete buffer;
            }
        }
    }

    void IPLayer::reassemble_packets() {
        u64 current_time = arch::CPU::read_tsc();
        u64 timeout = 30 * 1000 * 1000;
        
        for (auto it = fragment_buffers_.begin(); it != fragment_buffers_.end(); ) {
            FragmentBuffer* buffer = it->second;
            
            if (current_time - buffer->last_accessed > timeout) {
                delete buffer;
                it = fragment_buffers_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void IPLayer::cleanup_old_fragments() {
        reassemble_packets();
    }

    bool IPLayer::find_route(const IPAddress& destination, RouteEntry* route) {
        for (const RouteEntry& entry : routing_table_) {
            if (destination.is_ipv4 && entry.network.is_ipv4) {
                u32 dest_ip = destination.ipv4;
                u32 net_ip = entry.network.ipv4;
                u32 mask = entry.netmask.ipv4;
                
                if ((dest_ip & mask) == (net_ip & mask)) {
                    if (route) *route = entry;
                    return true;
                }
            }
        }
        
        return false;
    }

    u32 IPLayer::find_interface_for_address(const IPAddress& address) {
        for (usize i = 0; i < interfaces_.size(); i++) {
            if (interfaces_[i].address == address) {
                return i;
            }
        }
        return static_cast<u32>(-1);
    }

    void IPLayer::send_packet_to_interface(u32 interface_index, const IPAddress& destination,
                                          IPProtocol protocol, const u8* data, usize size) {
        if (interface_index >= interfaces_.size()) {
            return;
        }
        
        Interface& iface = interfaces_[interface_index];
        
        Vector<u8> packet(sizeof(IPv4Header) + size);
        IPv4Header* header = reinterpret_cast<IPv4Header*>(packet.data());
        
        header->version_ihl = (4 << 4) | 5;
        header->dscp_ecn = 0;
        header->total_length = swap_endian_16(sizeof(IPv4Header) + size);
        header->identification = swap_endian_16(identification_counter_++);
        header->flags_fragment_offset = 0;
        header->time_to_live = 64;
        header->protocol = static_cast<u8>(protocol);
        header->checksum = 0;
        header->source_address = iface.address.ipv4;
        header->destination_address = destination.ipv4;
        
        header->checksum = calculate_checksum(packet.data(), sizeof(IPv4Header));
        
        memcpy(packet.data() + sizeof(IPv4Header), data, size);
        
        u8 destination_mac[6];
        if (destination.ipv4 == IPAddress::broadcast().ipv4) {
            memset(destination_mac, 0xFF, 6);
        } else {
            memset(destination_mac, 0, 6);
        }
        
        iface.device->send(destination_mac, 
                          static_cast<u16>(EthernetHeader::Type::IPv4),
                          packet.data(), packet.size());
    }

    bool IPLayer::add_interface(u32 device_index, const IPAddress& address,
                               const IPAddress& netmask, const IPAddress& gateway) {
        ScopedLock lock(lock_);
        
        EthernetDevice* device = EthernetManager::instance().get_device(device_index);
        if (!device) {
            return false;
        }
        
        Interface iface;
        iface.index = interfaces_.size();
        iface.address = address;
        iface.netmask = netmask;
        iface.gateway = gateway;
        memcpy(iface.mac_address, device->get_mac_address(), 6);
        iface.is_up = true;
        iface.mtu = 1500;
        iface.device = device;
        
        interfaces_.push_back(iface);
        
        RouteEntry route;
        route.network = address;
        route.netmask = netmask;
        route.gateway = IPAddress::any();
        route.interface_index = iface.index;
        route.metric = 0;
        
        routing_table_.push_back(route);
        
        if (gateway.ipv4 != 0) {
            RouteEntry default_route;
            default_route.network = IPAddress::any();
            default_route.netmask = IPAddress::any();
            default_route.gateway = gateway;
            default_route.interface_index = iface.index;
            default_route.metric = 1;
            
            routing_table_.push_back(default_route);
        }
        
        debug::log(debug::LogLevel::Info, "IP",
                  "Added interface %u: %u.%u.%u.%u/%u.%u.%u.%u, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                  iface.index,
                  address.ipv4_bytes[0], address.ipv4_bytes[1],
                  address.ipv4_bytes[2], address.ipv4_bytes[3],
                  netmask.ipv4_bytes[0], netmask.ipv4_bytes[1],
                  netmask.ipv4_bytes[2], netmask.ipv4_bytes[3],
                  iface.mac_address[0], iface.mac_address[1],
                  iface.mac_address[2], iface.mac_address[3],
                  iface.mac_address[4], iface.mac_address[5]);
        
        return true;
    }

    bool IPLayer::remove_interface(u32 interface_index) {
        ScopedLock lock(lock_);
        
        if (interface_index >= interfaces_.size()) {
            return false;
        }
        
        interfaces_.erase(interface_index);
        
        for (usize i = 0; i < routing_table_.size(); i++) {
            if (routing_table_[i].interface_index == interface_index) {
                routing_table_.erase(i);
                i--;
            }
        }
        
        return true;
    }

    bool IPLayer::set_interface_address(u32 interface_index, const IPAddress& address) {
        ScopedLock lock(lock_);
        
        if (interface_index >= interfaces_.size()) {
            return false;
        }
        
        interfaces_[interface_index].address = address;
        return true;
    }

    bool IPLayer::set_interface_netmask(u32 interface_index, const IPAddress& netmask) {
        ScopedLock lock(lock_);
        
        if (interface_index >= interfaces_.size()) {
            return false;
        }
        
        interfaces_[interface_index].netmask = netmask;
        return true;
    }

    bool IPLayer::set_interface_gateway(u32 interface_index, const IPAddress& gateway) {
        ScopedLock lock(lock_);
        
        if (interface_index >= interfaces_.size()) {
            return false;
        }
        
        interfaces_[interface_index].gateway = gateway;
        return true;
    }

    const IPLayer::Interface* IPLayer::get_interface(u32 index) const {
        ScopedLock lock(lock_);
        
        if (index >= interfaces_.size()) {
            return nullptr;
        }
        
        return &interfaces_[index];
    }

    bool IPLayer::add_route(const IPAddress& network, const IPAddress& netmask,
                           const IPAddress& gateway, u32 interface_index, u32 metric) {
        ScopedLock lock(lock_);
        
        if (interface_index >= interfaces_.size()) {
            return false;
        }
        
        RouteEntry route;
        route.network = network;
        route.netmask = netmask;
        route.gateway = gateway;
        route.interface_index = interface_index;
        route.metric = metric;
        
        routing_table_.push_back(route);
        return true;
    }

    bool IPLayer::remove_route(const IPAddress& network, const IPAddress& netmask) {
        ScopedLock lock(lock_);
        
        for (usize i = 0; i < routing_table_.size(); i++) {
            if (routing_table_[i].network == network && 
                routing_table_[i].netmask == netmask) {
                routing_table_.erase(i);
                return true;
            }
        }
        
        return false;
    }

    bool IPLayer::send_packet(const IPAddress& destination, IPProtocol protocol,
                             const u8* data, usize size) {
        ScopedLock lock(lock_);
        
        RouteEntry route;
        if (!find_route(destination, &route)) {
            debug::log(debug::LogLevel::Error, "IP",
                      "No route to host: %u.%u.%u.%u",
                      destination.ipv4_bytes[0], destination.ipv4_bytes[1],
                      destination.ipv4_bytes[2], destination.ipv4_bytes[3]);
            return false;
        }
        
        IPAddress next_hop = route.gateway;
        if (next_hop.ipv4 == 0) {
            next_hop = destination;
        }
        
        send_packet_to_interface(route.interface_index, next_hop, protocol, data, size);
        
        debug::log(debug::LogLevel::Debug, "IP",
                  "Sent packet: dest=%u.%u.%u.%u, protocol=%u, size=%llu",
                  destination.ipv4_bytes[0], destination.ipv4_bytes[1],
                  destination.ipv4_bytes[2], destination.ipv4_bytes[3],
                  static_cast<u32>(protocol), size);
        
        return true;
    }

    bool IPLayer::register_protocol_handler(IPProtocol protocol, PacketCallback callback,
                                           void* user_data) {
        ScopedLock lock(callback_lock_);
        
        for (const ProtocolHandler& handler : protocol_handlers_) {
            if (handler.protocol == protocol && handler.callback == callback) {
                return false;
            }
        }
        
        ProtocolHandler handler;
        handler.protocol = protocol;
        handler.callback = callback;
        handler.user_data = user_data;
        
        protocol_handlers_.push_back(handler);
        return true;
    }

    bool IPLayer::unregister_protocol_handler(IPProtocol protocol, PacketCallback callback) {
        ScopedLock lock(callback_lock_);
        
        for (usize i = 0; i < protocol_handlers_.size(); i++) {
            if (protocol_handlers_[i].protocol == protocol && 
                protocol_handlers_[i].callback == callback) {
                protocol_handlers_.erase(i);
                return true;
            }
        }
        
        return false;
    }

    void IPLayer::process_packet(const u8* source_mac, const u8* destination_mac,
                                const u8* buffer, usize size) {
        if (!validate_packet(buffer, size)) {
            debug::log(debug::LogLevel::Warning, "IP", "Invalid IP packet");
            return;
        }
        
        const IPv4Header* header = reinterpret_cast<const IPv4Header*>(buffer);
        
        u16 total_length = swap_endian_16(header->total_length);
        if (total_length > size) {
            return;
        }
        
        u16 fragment_offset = header->fragment_offset();
        bool more_fragments = header->more_fragments();
        
        if (fragment_offset > 0 || more_fragments) {
            process_fragment(header, buffer, total_length);
            return;
        }
        
        IPPacket packet;
        packet.source.ipv4 = header->source_address;
        packet.destination.ipv4 = header->destination_address;
        packet.protocol = static_cast<IPProtocol>(header->protocol);
        packet.identification = swap_endian_16(header->identification);
        packet.time_to_live = header->time_to_live;
        
        usize data_offset = header->header_length();
        usize data_size = total_length - data_offset;
        
        packet.data.resize(data_size);
        memcpy(packet.data.data(), buffer + data_offset, data_size);
        packet.is_fragment = false;
        packet.fragment_offset = 0;
        packet.more_fragments = false;
        
        ScopedLock callback_lock(callback_lock_);
        for (const ProtocolHandler& handler : protocol_handlers_) {
            if (handler.protocol == packet.protocol) {
                handler.callback(packet, handler.user_data);
            }
        }
    }

    void IPLayer::poll() {
        cleanup_old_fragments();
    }

    void IPLayer::dump_interfaces() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "IP", "Network Interfaces:");
        for (const Interface& iface : interfaces_) {
            debug::log(debug::LogLevel::Info, "IP",
                      "  Interface %u:", iface.index);
            debug::log(debug::LogLevel::Info, "IP",
                      "    Address: %u.%u.%u.%u",
                      iface.address.ipv4_bytes[0], iface.address.ipv4_bytes[1],
                      iface.address.ipv4_bytes[2], iface.address.ipv4_bytes[3]);
            debug::log(debug::LogLevel::Info, "IP",
                      "    Netmask: %u.%u.%u.%u",
                      iface.netmask.ipv4_bytes[0], iface.netmask.ipv4_bytes[1],
                      iface.netmask.ipv4_bytes[2], iface.netmask.ipv4_bytes[3]);
            debug::log(debug::LogLevel::Info, "IP",
                      "    Gateway: %u.%u.%u.%u",
                      iface.gateway.ipv4_bytes[0], iface.gateway.ipv4_bytes[1],
                      iface.gateway.ipv4_bytes[2], iface.gateway.ipv4_bytes[3]);
            debug::log(debug::LogLevel::Info, "IP",
                      "    MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                      iface.mac_address[0], iface.mac_address[1],
                      iface.mac_address[2], iface.mac_address[3],
                      iface.mac_address[4], iface.mac_address[5]);
            debug::log(debug::LogLevel::Info, "IP",
                      "    Status: %s", iface.is_up ? "UP" : "DOWN");
            debug::log(debug::LogLevel::Info, "IP",
                      "    MTU: %u", iface.mtu);
        }
    }

    void IPLayer::dump_routing_table() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "IP", "Routing Table:");
        for (const RouteEntry& route : routing_table_) {
            debug::log(debug::LogLevel::Info, "IP",
                      "  %u.%u.%u.%u/%u.%u.%u.%u -> %u.%u.%u.%u via interface %u (metric: %u)",
                      route.network.ipv4_bytes[0], route.network.ipv4_bytes[1],
                      route.network.ipv4_bytes[2], route.network.ipv4_bytes[3],
                      route.netmask.ipv4_bytes[0], route.netmask.ipv4_bytes[1],
                      route.netmask.ipv4_bytes[2], route.netmask.ipv4_bytes[3],
                      route.gateway.ipv4_bytes[0], route.gateway.ipv4_bytes[1],
                      route.gateway.ipv4_bytes[2], route.gateway.ipv4_bytes[3],
                      route.interface_index, route.metric);
        }
    }

    void IPLayer::dump_fragment_buffers() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "IP", "Fragment Buffers: %llu", fragment_buffers_.size());
        for (const auto& pair : fragment_buffers_) {
            const FragmentBuffer* buffer = pair.second;
            debug::log(debug::LogLevel::Info, "IP",
                      "  Buffer: src=%u.%u.%u.%u, dst=%u.%u.%u.%u, id=%u, proto=%u, "
                      "frags=%llu, recv=%u, total=%u, complete=%s",
                      buffer->source.ipv4_bytes[0], buffer->source.ipv4_bytes[1],
                      buffer->source.ipv4_bytes[2], buffer->source.ipv4_bytes[3],
                      buffer->destination.ipv4_bytes[0], buffer->destination.ipv4_bytes[1],
                      buffer->destination.ipv4_bytes[2], buffer->destination.ipv4_bytes[3],
                      buffer->identification, static_cast<u32>(buffer->protocol),
                      buffer->fragments.size(), buffer->received_length,
                      buffer->total_length, buffer->is_complete ? "yes" : "no");
        }
    }

    IPLayer& IPLayer::instance() {
        static IPLayer instance;
        return instance;
    }
}
