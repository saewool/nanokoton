#ifndef NANOKOTON_UDP_HPP
#define NANOKOTON_UDP_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/net/ip.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/hashmap.hpp>
#include <nanokoton/lib/queue.hpp>
#include <nanokoton/lib/mutex.hpp>

namespace nk::net {
    struct PACKED UDPHeader {
        u16 source_port;
        u16 destination_port;
        u16 length;
        u16 checksum;
    };

    struct UDPDatagram {
        IPAddress source_address;
        u16 source_port;
        IPAddress destination_address;
        u16 destination_port;
        Vector<u8> data;
        u64 timestamp;
    };

    class UDPSocket {
    private:
        IPAddress local_address_;
        u16 local_port_;
        IPAddress remote_address_;
        u16 remote_port_;
        
        bool bound_;
        bool connected_;
        
        Queue<UDPDatagram> receive_queue_;
        Mutex queue_lock_;
        
        using ReceiveCallback = void (*)(const UDPDatagram& datagram, void* user_data);
        ReceiveCallback callback_;
        void* callback_user_data_;
        
    public:
        UDPSocket();
        ~UDPSocket();
        
        bool bind(const IPAddress& address, u16 port);
        bool connect(const IPAddress& address, u16 port);
        
        usize send(const u8* data, usize size);
        usize send_to(const IPAddress& address, u16 port, const u8* data, usize size);
        
        usize receive(u8* buffer, usize size, u64 timeout_ms = 0);
        usize receive_from(u8* buffer, usize size, IPAddress* source_address,
                          u16* source_port, u64 timeout_ms = 0);
        
        bool close();
        
        void set_receive_callback(ReceiveCallback callback, void* user_data) {
            callback_ = callback;
            callback_user_data_ = user_data;
        }
        
        bool is_bound() const { return bound_; }
        bool is_connected() const { return connected_; }
        
        u16 get_local_port() const { return local_port_; }
        const IPAddress& get_local_address() const { return local_address_; }
        u16 get_remote_port() const { return remote_port_; }
        const IPAddress& get_remote_address() const { return remote_address_; }
        
        void queue_datagram(const UDPDatagram& datagram);
        
        void poll();
    };
    
    class UDPLayer {
    private:
        struct SocketKey {
            IPAddress address;
            u16 port;
            
            bool operator==(const SocketKey& other) const {
                return address == other.address && port == other.port;
            }
            
            struct Hash {
                usize operator()(const SocketKey& key) const {
                    usize hash = key.address.is_ipv4 ? key.address.ipv4 : 
                                (key.address.ipv6[0] ^ key.address.ipv6[1]);
                    return hash * 31 + key.port;
                }
            };
        };
        
        HashMap<SocketKey, UDPSocket*, SocketKey::Hash> bound_sockets_;
        Mutex lock_;
        
        u16 allocate_port();
        bool is_port_available(u16 port);
        
        void process_udp_packet(const IPPacket& packet);
        
    public:
        UDPLayer();
        ~UDPLayer();
        
        bool init();
        
        UDPSocket* create_socket();
        bool bind_socket(UDPSocket* socket, const IPAddress& address, u16 port);
        bool close_socket(UDPSocket* socket);
        void remove_socket(UDPSocket* socket);
        
        void process_packet(const IPPacket& packet);
        
        usize get_bound_socket_count() const { return bound_sockets_.size(); }
        
        void dump_sockets() const;
        
        static UDPLayer& instance();
    };
}
#endif
