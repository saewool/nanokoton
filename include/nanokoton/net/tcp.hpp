#ifndef NANOKOTON_TCP_HPP
#define NANOKOTON_TCP_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/net/ip.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/hashmap.hpp>
#include <nanokoton/lib/queue.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/lib/ringbuffer.hpp>

namespace nk::net {
    struct PACKED TCPHeader {
        u16 source_port;
        u16 destination_port;
        u32 sequence_number;
        u32 acknowledgment_number;
        u8 data_offset_reserved;
        u8 flags;
        u16 window_size;
        u16 checksum;
        u16 urgent_pointer;
        
        u8 data_offset() const { return (data_offset_reserved >> 4) & 0x0F; }
        u16 header_length() const { return data_offset() * 4; }
        
        bool fin() const { return flags & 0x01; }
        bool syn() const { return flags & 0x02; }
        bool rst() const { return flags & 0x04; }
        bool psh() const { return flags & 0x08; }
        bool ack() const { return flags & 0x10; }
        bool urg() const { return flags & 0x20; }
        bool ece() const { return flags & 0x40; }
        bool cwr() const { return flags & 0x80; }
    };

    struct PACKED TCPPseudoHeader {
        u32 source_address;
        u32 destination_address;
        u8 zero;
        u8 protocol;
        u16 tcp_length;
    };

    enum class TCPState {
        Closed,
        Listen,
        SynSent,
        SynReceived,
        Established,
        FinWait1,
        FinWait2,
        CloseWait,
        Closing,
        LastAck,
        TimeWait
    };

    struct TCPSegment {
        u32 sequence_number;
        u32 acknowledgment_number;
        u16 window_size;
        bool syn;
        bool ack;
        bool fin;
        bool rst;
        Vector<u8> data;
        u64 timestamp;
    };

    class TCPSocket {
    private:
        struct SendBuffer {
            u32 sequence_start;
            u32 sequence_end;
            Vector<u8> data;
            u64 timestamp;
            bool acknowledged;
        };
        
        struct ReceiveBuffer {
            u32 sequence_start;
            u32 sequence_end;
            Vector<u8> data;
            bool consumed;
        };
        
        IPAddress local_address_;
        u16 local_port_;
        IPAddress remote_address_;
        u16 remote_port_;
        
        TCPState state_;
        u32 send_sequence_;
        u32 receive_sequence_;
        u32 send_unacknowledged_;
        u32 receive_next_expected_;
        
        u32 send_window_;
        u32 receive_window_;
        u32 maximum_segment_size_;
        
        Vector<SendBuffer> send_buffers_;
        Vector<ReceiveBuffer> receive_buffers_;
        RingBuffer<u8> receive_queue_;
        
        u64 last_activity_;
        u32 retransmit_timeout_;
        u32 retransmit_count_;
        
        Mutex send_lock_;
        Mutex receive_lock_;
        
        bool send_segment(const TCPSegment& segment);
        bool receive_segment(const TCPSegment& segment);
        
        void process_syn(const TCPSegment& segment);
        void process_syn_ack(const TCPSegment& segment);
        void process_ack(const TCPSegment& segment);
        void process_fin(const TCPSegment& segment);
        void process_rst(const TCPSegment& segment);
        void process_data(const TCPSegment& segment);
        
        void retransmit_pending_data();
        void acknowledge_data(u32 acknowledgment_number);
        void update_window(u32 window_size);
        
        bool validate_sequence(u32 sequence_number, u32 length);
        void reorder_buffers();
        void cleanup_acknowledged_data();
        
    public:
        TCPSocket();
        ~TCPSocket();
        
        bool bind(const IPAddress& address, u16 port);
        bool listen(u32 backlog = 5);
        bool connect(const IPAddress& address, u16 port);
        TCPSocket* accept();
        
        usize send(const u8* data, usize size);
        usize receive(u8* buffer, usize size, u64 timeout_ms = 0);
        
        bool close();
        bool abort();
        
        bool is_connected() const { return state_ == TCPState::Established; }
        bool is_listening() const { return state_ == TCPState::Listen; }
        bool is_closed() const { return state_ == TCPState::Closed; }
        
        TCPState get_state() const { return state_; }
        u16 get_local_port() const { return local_port_; }
        u16 get_remote_port() const { return remote_port_; }
        const IPAddress& get_local_address() const { return local_address_; }
        const IPAddress& get_remote_address() const { return remote_address_; }
        
        u32 get_send_window() const { return send_window_; }
        u32 get_receive_window() const { return receive_window_; }
        
        void poll();
        
        void dump_state() const;
    };
    
    class TCPLayer {
    private:
        struct ConnectionKey {
            IPAddress local_address;
            u16 local_port;
            IPAddress remote_address;
            u16 remote_port;
            
            bool operator==(const ConnectionKey& other) const {
                return local_address == other.local_address &&
                       local_port == other.local_port &&
                       remote_address == other.remote_address &&
                       remote_port == other.remote_port;
            }
            
            struct Hash {
                usize operator()(const ConnectionKey& key) const {
                    usize hash = key.local_address.is_ipv4 ? key.local_address.ipv4 : 
                                (key.local_address.ipv6[0] ^ key.local_address.ipv6[1]);
                    hash = hash * 31 + key.local_port;
                    hash = hash * 31 + (key.remote_address.is_ipv4 ? key.remote_address.ipv4 :
                                       (key.remote_address.ipv6[0] ^ key.remote_address.ipv6[1]));
                    hash = hash * 31 + key.remote_port;
                    return hash;
                }
            };
        };
        
        HashMap<ConnectionKey, TCPSocket*, ConnectionKey::Hash> connections_;
        HashMap<u16, TCPSocket*> listening_sockets_;
        Mutex lock_;
        
        u16 allocate_port();
        bool is_port_available(u16 port);
        
        void process_tcp_packet(const IPPacket& packet);
        void handle_new_connection(TCPSocket* listening_socket, const TCPSegment& segment);
        
    public:
        TCPLayer();
        ~TCPLayer();
        
        bool init();
        
        TCPSocket* create_socket();
        bool bind_socket(TCPSocket* socket, const IPAddress& address, u16 port);
        bool listen_socket(TCPSocket* socket, u32 backlog);
        bool connect_socket(TCPSocket* socket, const IPAddress& address, u16 port);
        TCPSocket* accept_socket(TCPSocket* socket);
        
        bool close_socket(TCPSocket* socket);
        void remove_socket(TCPSocket* socket);
        
        void process_packet(const IPPacket& packet);
        void poll_sockets();
        
        usize get_connection_count() const { return connections_.size(); }
        usize get_listening_socket_count() const { return listening_sockets_.size(); }
        
        void dump_connections() const;
        
        static TCPLayer& instance();
    };
}
#endif
