#ifndef NANOKOTON_ETHERNET_HPP
#define NANOKOTON_ETHERNET_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/queue.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/drivers/pci.hpp>

namespace nk::net {
    struct PACKED EthernetHeader {
        u8 destination[6];
        u8 source[6];
        u16 ether_type;
        
        enum Type : u16 {
            IPv4 = 0x0800,
            ARP = 0x0806,
            IPv6 = 0x86DD,
            VLAN = 0x8100,
            RARP = 0x8035
        };
    };

    class EthernetDevice {
    private:
        struct PACKED ReceiveDescriptor {
            u64 buffer_address;
            u64 buffer_address_high;
            u16 length;
            u16 checksum;
            u8 status;
            u8 errors;
            u16 vlan;
        };

        struct PACKED TransmitDescriptor {
            u64 buffer_address;
            u64 buffer_address_high;
            u16 length;
            u8 cso;
            u8 cmd;
            u8 status;
            u8 css;
            u16 vlan;
        };

        struct Buffer {
            u8* data;
            usize size;
            phys_addr physical;
        };

        PCI::Device* pci_device_;
        u8 mac_address_[6];
        u8* io_base_;
        u8* memory_base_;
        
        ReceiveDescriptor* rx_descriptors_;
        TransmitDescriptor* tx_descriptors_;
        Buffer* rx_buffers_;
        Buffer* tx_buffers_;
        
        u32 rx_index_;
        u32 tx_index_;
        u32 tx_clean_index_;
        
        Mutex tx_lock_;
        Mutex rx_lock_;
        
        bool promiscuous_;
        u32 interrupts_enabled_;
        u32 rx_buffer_size_;
        u32 tx_buffer_size_;
        u32 rx_descriptor_count_;
        u32 tx_descriptor_count_;
        
        bool init_hardware();
        bool init_descriptors();
        bool init_interrupts();
        
        bool receive_packet(u8* buffer, usize size);
        bool transmit_packet(const u8* buffer, usize size);
        
        void update_rx_descriptor(u32 index);
        void update_tx_descriptor(u32 index);
        void cleanup_tx_descriptors();
        
        bool set_mac_address(const u8* mac);
        bool get_mac_address(u8* mac);
        
        bool enable_multicast(const u8* address);
        bool disable_multicast(const u8* address);
        
        bool set_promiscuous(bool enable);
        bool set_multicast(bool enable);
        bool set_broadcast(bool enable);
        
        u32 read_register(u32 offset);
        void write_register(u32 offset, u32 value);
        
    public:
        EthernetDevice(PCI::Device* pci_device);
        ~EthernetDevice();
        
        bool init();
        bool start();
        bool stop();
        bool reset();
        
        bool send(const u8* destination, u16 ether_type, const u8* data, usize size);
        bool receive(u8* buffer, usize* size, u64 timeout_ms = 0);
        
        const u8* get_mac_address() const { return mac_address_; }
        bool set_mac_address(const u8* mac);
        
        bool is_promiscuous() const { return promiscuous_; }
        bool set_promiscuous_mode(bool enable);
        
        u32 get_mtu() const { return 1500; }
        u32 get_speed() const;
        bool is_link_up() const;
        
        bool get_statistics(u64* rx_packets, u64* tx_packets,
                           u64* rx_bytes, u64* tx_bytes,
                           u64* rx_errors, u64* tx_errors);
        
        void handle_interrupt();
        void poll();
        
        void dump_registers() const;
        void dump_statistics() const;
    };
    
    class EthernetManager {
    private:
        Vector<EthernetDevice*> devices_;
        Mutex lock_;
        
        struct Packet {
            u8 source[6];
            u8 destination[6];
            u16 ether_type;
            Vector<u8> data;
        };
        
        Queue<Packet> packet_queue_;
        Mutex queue_lock_;
        
        using ReceiveCallback = void (*)(const u8* source, const u8* destination,
                                        u16 ether_type, const u8* data, usize size,
                                        void* user_data);
        
        struct CallbackEntry {
            u16 ether_type;
            ReceiveCallback callback;
            void* user_data;
        };
        
        Vector<CallbackEntry> callbacks_;
        Mutex callback_lock_;
        
    public:
        EthernetManager();
        ~EthernetManager();
        
        bool init();
        bool add_device(PCI::Device* pci_device);
        EthernetDevice* get_device(u32 index);
        usize get_device_count() const { return devices_.size(); }
        
        bool send(u32 device_index, const u8* destination, u16 ether_type,
                 const u8* data, usize size);
        bool broadcast(u32 device_index, u16 ether_type,
                      const u8* data, usize size);
        
        bool register_callback(u16 ether_type, ReceiveCallback callback, void* user_data);
        bool unregister_callback(u16 ether_type, ReceiveCallback callback);
        
        void process_packets();
        void poll_devices();
        
        void dump_devices() const;
        
        static EthernetManager& instance();
    };
}
#endif
