#include <nanokoton/net/tcp.hpp>
#include <nanokoton/net/ip.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/algorithm.hpp>
#include <nanokoton/arch/cpu.hpp>

namespace nk::net {
    TCPSocket::TCPSocket()
        : local_port_(0),
          remote_port_(0),
          state_(TCPState::Closed),
          send_sequence_(0),
          receive_sequence_(0),
          send_unacknowledged_(0),
          receive_next_expected_(0),
          send_window_(65535),
          receive_window_(65535),
          maximum_segment_size_(1460),
          receive_queue_(8192),
          last_activity_(0),
          retransmit_timeout_(1000000),
          retransmit_count_(0) {
        last_activity_ = arch::CPU::read_tsc();
    }

    TCPSocket::~TCPSocket() {
        if (state_ != TCPState::Closed) {
            close();
        }
    }

    bool TCPSocket::bind(const IPAddress& address, u16 port) {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        if (state_ != TCPState::Closed) {
            return false;
        }
        
        local_address_ = address;
        local_port_ = port;
        
        TCPLayer& tcp_layer = TCPLayer::instance();
        if (!tcp_layer.bind_socket(this, address, port)) {
            return false;
        }
        
        return true;
    }

    bool TCPSocket::listen(u32 backlog) {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        if (state_ != TCPState::Closed) {
            return false;
        }
        
        state_ = TCPState::Listen;
        
        TCPLayer& tcp_layer = TCPLayer::instance();
        if (!tcp_layer.listen_socket(this, backlog)) {
            state_ = TCPState::Closed;
            return false;
        }
        
        return true;
    }

    bool TCPSocket::connect(const IPAddress& address, u16 port) {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        if (state_ != TCPState::Closed) {
            return false;
        }
        
        remote_address_ = address;
        remote_port_ = port;
        
        send_sequence_ = arch::CPU::read_tsc() & 0xFFFFFFFF;
        state_ = TCPState::SynSent;
        
        TCPSegment syn_segment;
        syn_segment.sequence_number = send_sequence_;
        syn_segment.acknowledgment_number = 0;
        syn_segment.window_size = receive_window_;
        syn_segment.syn = true;
        syn_segment.ack = false;
        syn_segment.fin = false;
        syn_segment.rst = false;
        syn_segment.timestamp = arch::CPU::read_tsc();
        
        if (!send_segment(syn_segment)) {
            state_ = TCPState::Closed;
            return false;
        }
        
        send_sequence_++;
        
        return true;
    }

    TCPSocket* TCPSocket::accept() {
        return nullptr;
    }

    usize TCPSocket::send(const u8* data, usize size) {
        ScopedLock lock(send_lock_);
        
        if (state_ != TCPState::Established) {
            return 0;
        }
        
        if (size == 0) {
            return 0;
        }
        
        usize sent = 0;
        while (sent < size) {
            usize chunk_size = min(size - sent, static_cast<usize>(maximum_segment_size_));
            
            SendBuffer buffer;
            buffer.sequence_start = send_sequence_;
            buffer.sequence_end = send_sequence_ + chunk_size;
            buffer.data.resize(chunk_size);
            memcpy(buffer.data.data(), data + sent, chunk_size);
            buffer.timestamp = arch::CPU::read_tsc();
            buffer.acknowledged = false;
            
            TCPSegment segment;
            segment.sequence_number = send_sequence_;
            segment.acknowledgment_number = receive_next_expected_;
            segment.window_size = receive_window_;
            segment.syn = false;
            segment.ack = true;
            segment.fin = false;
            segment.rst = false;
            segment.data.resize(chunk_size);
            memcpy(segment.data.data(), data + sent, chunk_size);
            segment.timestamp = buffer.timestamp;
            
            if (!send_segment(segment)) {
                break;
            }
            
            send_buffers_.push_back(buffer);
            send_sequence_ += chunk_size;
            sent += chunk_size;
            
            if (send_sequence_ - send_unacknowledged_ > send_window_) {
                break;
            }
        }
        
        return sent;
    }

    usize TCPSocket::receive(u8* buffer, usize size, u64 timeout_ms) {
        ScopedLock lock(receive_lock_);
        
        if (state_ != TCPState::Established) {
            return 0;
        }
        
        u64 start_time = arch::CPU::read_tsc();
        u64 timeout_cycles = timeout_ms * 1000000;
        
        while (receive_queue_.size() == 0) {
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
        
        usize to_read = min(size, receive_queue_.size());
        for (usize i = 0; i < to_read; i++) {
            buffer[i] = receive_queue_.pop();
        }
        
        receive_window_ = receive_queue_.capacity() - receive_queue_.size();
        
        if (to_read > 0) {
            TCPSegment ack_segment;
            ack_segment.sequence_number = send_sequence_;
            ack_segment.acknowledgment_number = receive_next_expected_;
            ack_segment.window_size = receive_window_;
            ack_segment.syn = false;
            ack_segment.ack = true;
            ack_segment.fin = false;
            ack_segment.rst = false;
            ack_segment.timestamp = arch::CPU::read_tsc();
            
            send_segment(ack_segment);
        }
        
        return to_read;
    }

    bool TCPSocket::close() {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        if (state_ == TCPState::Closed) {
            return true;
        }
        
        if (state_ == TCPState::Established) {
            state_ = TCPState::FinWait1;
            
            TCPSeggment fin_segment;
            fin_segment.sequence_number = send_sequence_;
            fin_segment.acknowledgment_number = receive_next_expected_;
            fin_segment.window_size = receive_window_;
            fin_segment.syn = false;
            fin_segment.ack = true;
            fin_segment.fin = true;
            fin_segment.rst = false;
            fin_segment.timestamp = arch::CPU::read_tsc();
            
            send_segment(fin_segment);
            send_sequence_++;
        } else {
            state_ = TCPState::Closed;
        }
        
        TCPLayer& tcp_layer = TCPLayer::instance();
        tcp_layer.remove_socket(this);
        
        return true;
    }

    bool TCPSocket::abort() {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        if (state_ == TCPState::Closed) {
            return true;
        }
        
        state_ = TCPState::Closed;
        
        TCPSegment rst_segment;
        rst_segment.sequence_number = send_sequence_;
        rst_segment.acknowledgment_number = receive_next_expected_;
        rst_segment.window_size = 0;
        rst_segment.syn = false;
        rst_segment.ack = false;
        rst_segment.fin = false;
        rst_segment.rst = true;
        rst_segment.timestamp = arch::CPU::read_tsc();
        
        send_segment(rst_segment);
        
        TCPLayer& tcp_layer = TCPLayer::instance();
        tcp_layer.remove_socket(this);
        
        return true;
    }

    void TCPSocket::poll() {
        ScopedLock lock1(send_lock_);
        ScopedLock lock2(receive_lock_);
        
        u64 current_time = arch::CPU::read_tsc();
        
        if (state_ == TCPState::Established) {
            retransmit_pending_data();
        }
        
        cleanup_acknowledged_data();
        reorder_buffers();
    }

    void TCPSocket::dump_state() const {
        debug::log(debug::LogLevel::Info, "TCP", "Socket State:");
        debug::log(debug::LogLevel::Info, "TCP", "  Local: %u.%u.%u.%u:%u",
                  local_address_.ipv4_bytes[0], local_address_.ipv4_bytes[1],
                  local_address_.ipv4_bytes[2], local_address_.ipv4_bytes[3],
                  local_port_);
        debug::log(debug::LogLevel::Info, "TCP", "  Remote: %u.%u.%u.%u:%u",
                  remote_address_.ipv4_bytes[0], remote_address_.ipv4_bytes[1],
                  remote_address_.ipv4_bytes[2], remote_address_.ipv4_bytes[3],
                  remote_port_);
        debug::log(debug::LogLevel::Info, "TCP", "  State: %u", static_cast<u32>(state_));
        debug::log(debug::LogLevel::Info, "TCP", "  Send Seq: %u, Recv Seq: %u",
                  send_sequence_, receive_sequence_);
        debug::log(debug::LogLevel::Info, "TCP", "  Send Window: %u, Recv Window: %u",
                  send_window_, receive_window_);
        debug::log(debug::LogLevel::Info, "TCP", "  Send Buffers: %llu, Recv Queue: %llu/%llu",
                  send_buffers_.size(), receive_queue_.size(), receive_queue_.capacity());
    }

    bool TCPSocket::send_segment(const TCPSegment& segment) {
        Vector<u8> packet(sizeof(TCPHeader) + segment.data.size());
        TCPHeader* header = reinterpret_cast<TCPHeader*>(packet.data());
        
        header->source_port = swap_endian_16(local_port_);
        header->destination_port = swap_endian_16(remote_port_);
        header->sequence_number = swap_endian_32(segment.sequence_number);
        header->acknowledgment_number = swap_endian_32(segment.acknowledgment_number);
        
        u8 data_offset = 5;
        u8 flags = 0;
        if (segment.syn) flags |= 0x02;
        if (segment.ack) flags |= 0x10;
        if (segment.fin) flags |= 0x01;
        if (segment.rst) flags |= 0x04;
        
        header->data_offset_reserved = (data_offset << 4);
        header->flags = flags;
        header->window_size = swap_endian_16(segment.window_size);
        header->checksum = 0;
        header->urgent_pointer = 0;
        
        memcpy(packet.data() + sizeof(TCPHeader), segment.data.data(), segment.data.size());
        
        TCPPseudoHeader pseudo;
        pseudo.source_address = local_address_.ipv4;
        pseudo.destination_address = remote_address_.ipv4;
        pseudo.zero = 0;
        pseudo.protocol = static_cast<u8>(IPProtocol::TCP);
        pseudo.tcp_length = swap_endian_16(sizeof(TCPHeader) + segment.data.size());
        
        Vector<u8> checksum_data(sizeof(TCPPseudoHeader) + packet.size());
        memcpy(checksum_data.data(), &pseudo, sizeof(TCPPseudoHeader));
        memcpy(checksum_data.data() + sizeof(TCPPseudoHeader), packet.data(), packet.size());
        
        header->checksum = IPLayer::instance().calculate_checksum(
            checksum_data.data(), checksum_data.size());
        
        IPLayer& ip_layer = IPLayer::instance();
        return ip_layer.send_packet(remote_address_, IPProtocol::TCP,
                                   packet.data(), packet.size());
    }

    bool TCPSocket::receive_segment(const TCPSegment& segment) {
        if (!validate_sequence(segment.sequence_number, segment.data.size())) {
            return false;
        }
        
        if (segment.syn) {
            process_syn(segment);
        } else if (segment.ack) {
            process_ack(segment);
        } else if (segment.fin) {
            process_fin(segment);
        } else if (segment.rst) {
            process_rst(segment);
        } else if (segment.data.size() > 0) {
            process_data(segment);
        }
        
        last_activity_ = arch::CPU::read_tsc();
        return true;
    }

    void TCPSocket::process_syn(const TCPSegment& segment) {
        if (state_ == TCPState::Listen) {
            state_ = TCPState::SynReceived;
            remote_address_ = segment.source_address;
            remote_port_ = segment.source_port;
            receive_sequence_ = segment.sequence_number + 1;
            receive_next_expected_ = receive_sequence_;
            
            TCPSegment syn_ack;
            syn_ack.sequence_number = send_sequence_;
            syn_ack.acknowledgment_number = receive_sequence_;
            syn_ack.window_size = receive_window_;
            syn_ack.syn = true;
            syn_ack.ack = true;
            syn_ack.fin = false;
            syn_ack.rst = false;
            syn_ack.timestamp = arch::CPU::read_tsc();
            
            send_segment(syn_ack);
            send_sequence_++;
        }
    }

    void TCPSocket::process_ack(const TCPSegment& segment) {
        if (segment.acknowledgment_number > send_unacknowledged_) {
            acknowledge_data(segment.acknowledgment_number);
        }
        
        if (segment.window_size != send_window_) {
            update_window(segment.window_size);
        }
    }

    void TCPSocket::process_fin(const TCPSegment& segment) {
        if (state_ == TCPState::Established) {
            state_ = TCPState::CloseWait;
            receive_sequence_++;
            
            TCPSegment ack;
            ack.sequence_number = send_sequence_;
            ack.acknowledgment_number = receive_sequence_;
            ack.window_size = receive_window_;
            ack.syn = false;
            ack.ack = true;
            ack.fin = false;
            ack.rst = false;
            ack.timestamp = arch::CPU::read_tsc();
            
            send_segment(ack);
        }
    }

    void TCPSocket::process_rst(const TCPSegment& segment) {
        state_ = TCPState::Closed;
    }

    void TCPSocket::process_data(const TCPSegment& segment) {
        ReceiveBuffer buffer;
        buffer.sequence_start = segment.sequence_number;
        buffer.sequence_end = segment.sequence_number + segment.data.size();
        buffer.data = segment.data;
        buffer.consumed = false;
        
        receive_buffers_.push_back(buffer);
        reorder_buffers();
        
        receive_sequence_ = segment.sequence_number + segment.data.size();
        
        TCPSegment ack;
        ack.sequence_number = send_sequence_;
        ack.acknowledgment_number = receive_sequence_;
        ack.window_size = receive_window_;
        ack.syn = false;
        ack.ack = true;
        ack.fin = false;
        ack.rst = false;
        ack.timestamp = arch::CPU::read_tsc();
        
        send_segment(ack);
    }

    void TCPSocket::retransmit_pending_data() {
        u64 current_time = arch::CPU::read_tsc();
        
        for (SendBuffer& buffer : send_buffers_) {
            if (!buffer.acknowledged && 
                current_time - buffer.timestamp > retransmit_timeout_) {
                
                if (retransmit_count_ > 10) {
                    abort();
                    return;
                }
                
                TCPSegment segment;
                segment.sequence_number = buffer.sequence_start;
                segment.acknowledgment_number = receive_next_expected_;
                segment.window_size = receive_window_;
                segment.syn = false;
                segment.ack = true;
                segment.fin = false;
                segment.rst = false;
                segment.data = buffer.data;
                segment.timestamp = current_time;
                
                send_segment(segment);
                buffer.timestamp = current_time;
                retransmit_count_++;
            }
        }
    }

    void TCPSocket::acknowledge_data(u32 acknowledgment_number) {
        for (SendBuffer& buffer : send_buffers_) {
            if (buffer.sequence_end <= acknowledgment_number) {
                buffer.acknowledged = true;
            }
        }
        send_unacknowledged_ = acknowledgment_number;
    }

    void TCPSocket::update_window(u32 window_size) {
        send_window_ = window_size;
    }

    bool TCPSocket::validate_sequence(u32 sequence_number, u32 length) {
        if (length == 0) {
            return true;
        }
        
        if (sequence_number >= receive_sequence_ && 
            sequence_number + length <= receive_sequence_ + receive_window_) {
            return true;
        }
        
        return false;
    }

    void TCPSocket::reorder_buffers() {
        sort(receive_buffers_.begin(), receive_buffers_.end(),
             [](const ReceiveBuffer& a, const ReceiveBuffer& b) {
                 return a.sequence_start < b.sequence_start;
             });
        
        for (const ReceiveBuffer& buffer : receive_buffers_) {
            if (buffer.sequence_start == receive_next_expected_ && !buffer.consumed) {
                for (u8 byte : buffer.data) {
                    if (receive_queue_.full()) {
                        break;
                    }
                    receive_queue_.push(byte);
                }
                receive_next_expected_ = buffer.sequence_end;
                const_cast<ReceiveBuffer&>(buffer).consumed = true;
            }
        }
    }

    void TCPSocket::cleanup_acknowledged_data() {
        for (usize i = 0; i < send_buffers_.size(); i++) {
            if (send_buffers_[i].acknowledged) {
                send_buffers_.erase(i);
                i--;
            }
        }
        
        for (usize i = 0; i < receive_buffers_.size(); i++) {
            if (receive_buffers_[i].consumed) {
                receive_buffers_.erase(i);
                i--;
            }
        }
    }

    TCPLayer::TCPLayer() {
        debug::log(debug::LogLevel::Info, "TCP", "TCP Layer created");
    }

    TCPLayer::~TCPLayer() {
        ScopedLock lock(lock_);
        
        for (auto& pair : connections_) {
            delete pair.second;
        }
        connections_.clear();
        
        for (auto& pair : listening_sockets_) {
            delete pair.second;
        }
        listening_sockets_.clear();
    }

    bool TCPLayer::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "TCP", "Initializing TCP Layer");
        
        IPLayer& ip_layer = IPLayer::instance();
        ip_layer.register_protocol_handler(IPProtocol::TCP,
                                          [](const IPPacket& packet, void* user_data) {
            TCPLayer* tcp = static_cast<TCPLayer*>(user_data);
            tcp->process_packet(packet);
        }, this);
        
        debug::log(debug::LogLevel::Info, "TCP", "TCP Layer initialized");
        return true;
    }

    u16 TCPLayer::allocate_port() {
        static u16 next_port = 1024;
        
        while (is_port_available(next_port)) {
            next_port++;
            if (next_port < 1024 || next_port > 65535) {
                next_port = 1024;
            }
        }
        
        return next_port++;
    }

    bool TCPLayer::is_port_available(u16 port) {
        for (const auto& pair : connections_) {
            if (pair.first.local_port == port || pair.first.remote_port == port) {
                return false;
            }
        }
        
        for (const auto& pair : listening_sockets_) {
            if (pair.first == port) {
                return false;
            }
        }
        
        return true;
    }

    void TCPLayer::process_tcp_packet(const IPPacket& packet) {
        if (packet.data.size() < sizeof(TCPHeader)) {
            return;
        }
        
        const TCPHeader* header = reinterpret_cast<const TCPHeader*>(packet.data.data());
        
        u16 source_port = swap_endian_16(header->source_port);
        u16 dest_port = swap_endian_16(header->destination_port);
        u32 sequence = swap_endian_32(header->sequence_number);
        u32 ack = swap_endian_32(header->acknowledgment_number);
        
        ConnectionKey key;
        key.local_address = packet.destination;
        key.local_port = dest_port;
        key.remote_address = packet.source;
        key.remote_port = source_port;
        
        TCPSocket* socket = nullptr;
        auto it = connections_.find(key);
        
        if (it != connections_.end()) {
            socket = it->second;
        } else {
            auto listen_it = listening_sockets_.find(dest_port);
            if (listen_it != listening_sockets_.end()) {
                handle_new_connection(listen_it->second, packet);
                return;
            }
            return;
        }
        
        TCPSegment segment;
        segment.source_address = packet.source;
        segment.source_port = source_port;
        segment.destination_address = packet.destination;
        segment.destination_port = dest_port;
        segment.sequence_number = sequence;
        segment.acknowledgment_number = ack;
        segment.window_size = swap_endian_16(header->window_size);
        segment.syn = header->syn();
        segment.ack = header->ack();
        segment.fin = header->fin();
        segment.rst = header->rst();
        
        usize data_offset = header->data_offset() * 4;
        usize data_size = packet.data.size() - data_offset;
        
        if (data_size > 0) {
            segment.data.resize(data_size);
            memcpy(segment.data.data(), packet.data.data() + data_offset, data_size);
        }
        
        segment.timestamp = arch::CPU::read_tsc();
        
        socket->receive_segment(segment);
    }

    void TCPLayer::handle_new_connection(TCPSocket* listening_socket, const IPPacket& packet) {
        const TCPHeader* header = reinterpret_cast<const TCPHeader*>(packet.data.data());
        
        if (!header->syn()) {
            return;
        }
        
        u16 source_port = swap_endian_16(header->source_port);
        u16 dest_port = swap_endian_16(header->destination_port);
        
        TCPSocket* new_socket = new TCPSocket();
        new_socket->local_address_ = packet.destination;
        new_socket->local_port_ = dest_port;
        new_socket->remote_address_ = packet.source;
        new_socket->remote_port_ = source_port;
        new_socket->state_ = TCPState::SynReceived;
        
        ConnectionKey key;
        key.local_address = packet.destination;
        key.local_port = dest_port;
        key.remote_address = packet.source;
        key.remote_port = source_port;
        
        connections_[key] = new_socket;
        
        TCPSegment segment;
        segment.source_address = packet.source;
        segment.source_port = source_port;
        segment.destination_address = packet.destination;
        segment.destination_port = dest_port;
        segment.sequence_number = swap_endian_32(header->sequence_number);
        segment.acknowledgment_number = 0;
        segment.window_size = swap_endian_16(header->window_size);
        segment.syn = true;
        segment.ack = false;
        segment.fin = false;
        segment.rst = false;
        segment.timestamp = arch::CPU::read_tsc();
        
        new_socket->receive_segment(segment);
    }

    TCPSocket* TCPLayer::create_socket() {
        return new TCPSocket();
    }

    bool TCPLayer::bind_socket(TCPSocket* socket, const IPAddress& address, u16 port) {
        ScopedLock lock(lock_);
        
        if (port == 0) {
            port = allocate_port();
        }
        
        if (!is_port_available(port)) {
            return false;
        }
        
        socket->local_address_ = address;
        socket->local_port_ = port;
        
        return true;
    }

    bool TCPLayer::listen_socket(TCPSocket* socket, u32 backlog) {
        ScopedLock lock(lock_);
        
        listening_sockets_[socket->local_port_] = socket;
        return true;
    }

    bool TCPLayer::connect_socket(TCPSocket* socket, const IPAddress& address, u16 port) {
        ScopedLock lock(lock_);
        
        ConnectionKey key;
        key.local_address = socket->local_address_;
        key.local_port = socket->local_port_;
        key.remote_address = address;
        key.remote_port = port;
        
        connections_[key] = socket;
        return socket->connect(address, port);
    }

    TCPSocket* TCPLayer::accept_socket(TCPSocket* socket) {
        return nullptr;
    }

    bool TCPLayer::close_socket(TCPSocket* socket) {
        ScopedLock lock(lock_);
        
        return socket->close();
    }

    void TCPLayer::remove_socket(TCPSocket* socket) {
        ScopedLock lock(lock_);
        
        for (auto it = connections_.begin(); it != connections_.end(); ++it) {
            if (it->second == socket) {
                connections_.erase(it);
                break;
            }
        }
        
        for (auto it = listening_sockets_.begin(); it != listening_sockets_.end(); ++it) {
            if (it->second == socket) {
                listening_sockets_.erase(it);
                break;
            }
        }
    }

    void TCPLayer::process_packet(const IPPacket& packet) {
        ScopedLock lock(lock_);
        process_tcp_packet(packet);
    }

    void TCPLayer::poll_sockets() {
        ScopedLock lock(lock_);
        
        for (auto& pair : connections_) {
            pair.second->poll();
        }
    }

    void TCPLayer::dump_connections() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "TCP", "TCP Connections: %llu", connections_.size());
        for (const auto& pair : connections_) {
            const TCPSocket* socket = pair.second;
            debug::log(debug::LogLevel::Info, "TCP",
                      "  %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u, State: %u",
                      socket->local_address_.ipv4_bytes[0], socket->local_address_.ipv4_bytes[1],
                      socket->local_address_.ipv4_bytes[2], socket->local_address_.ipv4_bytes[3],
                      socket->local_port_,
                      socket->remote_address_.ipv4_bytes[0], socket->remote_address_.ipv4_bytes[1],
                      socket->remote_address_.ipv4_bytes[2], socket->remote_address_.ipv4_bytes[3],
                      socket->remote_port_,
                      static_cast<u32>(socket->state_));
        }
        
        debug::log(debug::LogLevel::Info, "TCP", "Listening Sockets: %llu", listening_sockets_.size());
        for (const auto& pair : listening_sockets_) {
            const TCPSocket* socket = pair.second;
            debug::log(debug::LogLevel::Info, "TCP",
                      "  Port: %u, Address: %u.%u.%u.%u",
                      pair.first,
                      socket->local_address_.ipv4_bytes[0], socket->local_address_.ipv4_bytes[1],
                      socket->local_address_.ipv4_bytes[2], socket->local_address_.ipv4_bytes[3]);
        }
    }

    TCPLayer& TCPLayer::instance() {
        static TCPLayer instance;
        return instance;
    }
}
