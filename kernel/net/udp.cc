#include <nanokoton/net/udp.hpp>
#include <nanokoton/net/ip.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/algorithm.hpp>
#include <nanokoton/arch/cpu.hpp>

namespace nk::net {
    UDPSocket::UDPSocket()
        : local_port_(0),
          remote_port_(0),
          bound_(false),
          connected_(false),
          callback_(nullptr),
          callback_user_data_(nullptr) {}

    UDPSocket::~UDPSocket() {
        close();
    }

    bool UDPSocket::bind(const IPAddress& address, u16 port) {
        ScopedLock lock(queue_lock_);
        
        if (bound_) {
            return false;
        }
        
        local_address_ = address;
        local_port_ = port;
        bound_ = true;
        
        UDPLayer& udp_layer = UDPLayer::instance();
        if (!udp_layer.bind_socket(this, address, port)) {
            bound_ = false;
            return false;
        }
        
        return true;
    }

    bool UDPSocket::connect(const IPAddress& address, u16 port) {
        ScopedLock lock(queue_lock_);
        
        if (!bound_) {
            return false;
        }
        
        remote_address_ = address;
        remote_port_ = port;
        connected_ = true;
        
        return true;
    }

    usize UDPSocket::send(const u8* data, usize size) {
        if (!connected_) {
            return 0;
        }
        
        return send_to(remote_address_, remote_port_, data, size);
    }

    usize UDPSocket::send_to(const IPAddress& address, u16 port, const u8* data, usize size) {
        ScopedLock lock(queue_lock_);
        
        if (!bound_) {
            return 0;
        }
        
        if (size > 65507) {
            return 0;
        }
        
        Vector<u8> packet(sizeof(UDPHeader) + size);
        UDPHeader* header = reinterpret_cast<UDPHeader*>(packet.data());
        
        header->source_port = swap_endian_16(local_port_);
        header->destination_port = swap_endian_16(port);
        header->length = swap_endian_16(sizeof(UDPHeader) + size);
        header->checksum = 0;
        
        memcpy(packet.data() + sizeof(UDPHeader), data, size);
        
        IPLayer& ip_layer = IPLayer::instance();
        if (!ip_layer.send_packet(address, IPProtocol::UDP, packet.data(), packet.size())) {
            return 0;
        }
        
        return size;
    }

    usize UDPSocket::receive(u8* buffer, usize size, u64 timeout_ms) {
        IPAddress source_addr;
        u16 source_port;
        return receive_from(buffer, size, &source_addr, &source_port, timeout_ms);
    }

    usize UDPSocket::receive_from(u8* buffer, usize size, IPAddress* source_address,
                                 u16* source_port, u64 timeout_ms) {
        ScopedLock lock(queue_lock_);
        
        if (!bound_) {
            return 0;
        }
        
        u64 start_time = arch::CPU::read_tsc();
        u64 timeout_cycles = timeout_ms * 1000000;
        
        while (receive_queue_.empty()) {
            if (timeout_ms != 0) {
                u64 current_time = arch::CPU::read_tsc();
                if (current_time - start_time > timeout_cycles) {
                    return 0;
                }
            }
            
            lock.unlock();
            arch::CPU::pause();
            lock.lock();
        }
        
        UDPDatagram datagram = receive_queue_.front();
        receive_queue_.pop();
        
        usize to_copy = min(size, datagram.data.size());
        memcpy(buffer, datagram.data.data(), to_copy);
        
        if (source_address) {
            *source_address = datagram.source_address;
        }
        
        if (source_port) {
            *source_port = datagram.source_port;
        }
        
        return to_copy;
    }

    bool UDPSocket::close() {
        ScopedLock lock(queue_lock_);
        
        if (!bound_) {
            return true;
        }
        
        UDPLayer& udp_layer = UDPLayer::instance();
        udp_layer.remove_socket(this);
        
        bound_ = false;
        connected_ = false;
        
        while (!receive_queue_.empty()) {
            receive_queue_.pop();
        }
        
        return true;
    }

    void UDPSocket::queue_datagram(const UDPDatagram& datagram) {
        ScopedLock lock(queue_lock_);
        
        if (!bound_) {
            return;
        }
        
        if (datagram.destination_port != local_port_) {
            return;
        }
        
        if (!connected_ || (connected_ && datagram.source_address == remote_address_ &&
                          datagram.source_port == remote_port_)) {
            receive_queue_.push(datagram);
            
            if (callback_) {
                callback_(datagram, callback_user_data_);
            }
        }
    }

    void UDPSocket::poll() {
        // UDP sockets are connectionless, polling is handled by the UDP layer
    }

    UDPLayer::UDPLayer() {
        debug::log(debug::LogLevel::Info, "UDP", "UDP Layer created");
    }

    UDPLayer::~UDPLayer() {
        ScopedLock lock(lock_);
        
        for (auto& pair : bound_sockets_) {
            delete pair.second;
        }
        bound_sockets_.clear();
    }

    bool UDPLayer::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "UDP", "Initializing UDP Layer");
        
        IPLayer& ip_layer = IPLayer::instance();
        ip_layer.register_protocol_handler(IPProtocol::UDP,
                                          [](const IPPacket& packet, void* user_data) {
            UDPLayer* udp = static_cast<UDPLayer*>(user_data);
            udp->process_packet(packet);
        }, this);
        
        debug::log(debug::LogLevel::Info, "UDP", "UDP Layer initialized");
        return true;
    }

    u16 UDPLayer::allocate_port() {
        static u16 next_port = 1024;
        
        while (!is_port_available(next_port)) {
            next_port++;
            if (next_port < 1024 || next_port > 65535) {
                next_port = 1024;
            }
        }
        
        return next_port;
    }

    bool UDPLayer::is_port_available(u16 port) {
        ScopedLock lock(lock_);
        
        SocketKey key;
        key.port = port;
        
        return bound_sockets_.find(key) == bound_sockets_.end();
    }

    void UDPLayer::process_udp_packet(const IPPacket& packet) {
        if (packet.data.size() < sizeof(UDPHeader)) {
            return;
        }
        
        const UDPHeader* header = reinterpret_cast<const UDPHeader*>(packet.data.data());
        
        u16 source_port = swap_endian_16(header->source_port);
        u16 dest_port = swap_endian_16(header->destination_port);
        u16 length = swap_endian_16(header->length);
        
        if (length < sizeof(UDPHeader) || length > packet.data.size()) {
            return;
        }
        
        SocketKey key;
        key.address = packet.destination;
        key.port = dest_port;
        
        ScopedLock lock(lock_);
        auto it = bound_sockets_.find(key);
        if (it == bound_sockets_.end()) {
            return;
        }
        
        UDPSocket* socket = it->second;
        
        UDPDatagram datagram;
        datagram.source_address = packet.source;
        datagram.source_port = source_port;
        datagram.destination_address = packet.destination;
        datagram.destination_port = dest_port;
        datagram.timestamp = arch::CPU::read_tsc();
        
        usize data_size = length - sizeof(UDPHeader);
        datagram.data.resize(data_size);
        memcpy(datagram.data.data(), packet.data.data() + sizeof(UDPHeader), data_size);
        
        socket->queue_datagram(datagram);
    }

    UDPSocket* UDPLayer::create_socket() {
        return new UDPSocket();
    }

    bool UDPLayer::bind_socket(UDPSocket* socket, const IPAddress& address, u16 port) {
        ScopedLock lock(lock_);
        
        if (port == 0) {
            port = allocate_port();
        }
        
        SocketKey key;
        key.address = address;
        key.port = port;
        
        if (bound_sockets_.find(key) != bound_sockets_.end()) {
            return false;
        }
        
        bound_sockets_[key] = socket;
        return true;
    }

    bool UDPLayer::close_socket(UDPSocket* socket) {
        ScopedLock lock(lock_);
        
        return socket->close();
    }

    void UDPLayer::remove_socket(UDPSocket* socket) {
        ScopedLock lock(lock_);
        
        for (auto it = bound_sockets_.begin(); it != bound_sockets_.end(); ++it) {
            if (it->second == socket) {
                bound_sockets_.erase(it);
                break;
            }
        }
    }

    void UDPLayer::process_packet(const IPPacket& packet) {
        ScopedLock lock(lock_);
        process_udp_packet(packet);
    }

    void UDPLayer::dump_sockets() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "UDP", "UDP Bound Sockets: %llu", bound_sockets_.size());
        for (const auto& pair : bound_sockets_) {
            const UDPSocket* socket = pair.second;
            debug::log(debug::LogLevel::Info, "UDP",
                      "  %u.%u.%u.%u:%u, Connected: %s",
                      pair.first.address.ipv4_bytes[0], pair.first.address.ipv4_bytes[1],
                      pair.first.address.ipv4_bytes[2], pair.first.address.ipv4_bytes[3],
                      pair.first.port,
                      socket->is_connected() ? "yes" : "no");
        }
    }

    UDPLayer& UDPLayer::instance() {
        static UDPLayer instance;
        return instance;
    }
}
