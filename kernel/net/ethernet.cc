#include <nanokoton/net/ethernet.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/mm/physical.hpp>
#include <nanokoton/drivers/pci.hpp>
#include <nanokoton/arch/io.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/bitops.hpp>
#include <nanokoton/lib/algorithm.hpp>

namespace nk::net {
    EthernetDevice::EthernetDevice(PCI::Device* pci_device)
        : pci_device_(pci_device),
          io_base_(nullptr),
          memory_base_(nullptr),
          rx_descriptors_(nullptr),
          tx_descriptors_(nullptr),
          rx_buffers_(nullptr),
          tx_buffers_(nullptr),
          rx_index_(0),
          tx_index_(0),
          tx_clean_index_(0),
          promiscuous_(false),
          interrupts_enabled_(0),
          rx_buffer_size_(2048),
          tx_buffer_size_(2048),
          rx_descriptor_count_(256),
          tx_descriptor_count_(256) {
        
        memset(mac_address_, 0, sizeof(mac_address_));
    }

    EthernetDevice::~EthernetDevice() {
        stop();
        
        if (rx_buffers_) {
            for (u32 i = 0; i < rx_descriptor_count_; i++) {
                if (rx_buffers_[i].data) {
                    mm::VirtualMemoryManager::instance().kfree(rx_buffers_[i].data);
                }
            }
            delete[] rx_buffers_;
        }
        
        if (tx_buffers_) {
            for (u32 i = 0; i < tx_descriptor_count_; i++) {
                if (tx_buffers_[i].data) {
                    mm::VirtualMemoryManager::instance().kfree(tx_buffers_[i].data);
                }
            }
            delete[] tx_buffers_;
        }
        
        if (rx_descriptors_) {
            mm::VirtualMemoryManager::instance().kfree(rx_descriptors_);
        }
        
        if (tx_descriptors_) {
            mm::VirtualMemoryManager::instance().kfree(tx_descriptors_);
        }
        
        if (memory_base_) {
            mm::VirtualMemoryManager::instance().kfree(memory_base_);
        }
    }

    bool EthernetDevice::init() {
        if (!pci_device_) {
            return false;
        }

        pci_device_->enable_bus_mastering();
        pci_device_->enable_memory_space();
        pci_device_->enable_io_space();

        u32 bar0 = pci_device_->get_bar(0);
        u32 bar1 = pci_device_->get_bar(1);

        if (bar0 & 1) {
            io_base_ = reinterpret_cast<u8*>(bar0 & ~3);
        } else {
            memory_base_ = reinterpret_cast<u8*>(
                mm::VirtualMemoryManager::instance().kmalloc_aligned(8192, 4096));
            
            if (!memory_base_) {
                debug::log(debug::LogLevel::Error, "ETH", 
                          "Failed to allocate memory for device registers");
                return false;
            }

            phys_addr mem_phys = bar0 & ~0xF;
            if (!mm::VirtualMemoryManager::instance().map_page(
                    reinterpret_cast<virt_addr>(memory_base_), mem_phys,
                    mm::PageFlags::Present | mm::PageFlags::Writable | 
                    mm::PageFlags::CacheDisabled)) {
                debug::log(debug::LogLevel::Error, "ETH",
                          "Failed to map device registers");
                mm::VirtualMemoryManager::instance().kfree(memory_base_);
                memory_base_ = nullptr;
                return false;
            }
        }

        if (!init_hardware()) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to initialize hardware");
            return false;
        }

        if (!get_mac_address(mac_address_)) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to read MAC address");
            return false;
        }

        debug::log(debug::LogLevel::Info, "ETH",
                  "Ethernet device initialized, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac_address_[0], mac_address_[1], mac_address_[2],
                  mac_address_[3], mac_address_[4], mac_address_[5]);

        return true;
    }

    bool EthernetDevice::init_hardware() {
        write_register(0x0000, 0x00000004);

        u32 timeout = 1000;
        while (timeout--) {
            if (!(read_register(0x0000) & 0x00000004)) {
                break;
            }
            arch::CPU::pause();
        }

        if (read_register(0x0000) & 0x00000004) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Device reset timeout");
            return false;
        }

        u32 eecd = read_register(0x0010);
        eecd |= 0x00000004;
        write_register(0x0010, eecd);

        write_register(0x0028, 0x00000000);
        write_register(0x002C, 0x00000000);

        write_register(0x0100, 0x00000000);
        write_register(0x0108, 0x00000000);

        interrupts_enabled_ = 0;
        write_register(0x00D0, 0xFFFFFFFF);
        write_register(0x00D4, 0xFFFFFFFF);
        write_register(0x00D8, 0xFFFFFFFF);
        write_register(0x00DC, 0xFFFFFFFF);

        write_register(0x0280, 0x00000000);

        if (!init_descriptors()) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to initialize descriptors");
            return false;
        }

        u32 rctl = read_register(0x0100);
        rctl |= 0x00000002;
        write_register(0x0100, rctl);

        u32 tctl = read_register(0x0400);
        tctl |= 0x00000002;
        tctl |= 0x00000010;
        tctl |= 0x00000100;
        write_register(0x0400, tctl);

        set_promiscuous(false);

        u32 rxcsum = read_register(0x0500);
        rxcsum |= 0x00000001;
        write_register(0x0500, rxcsum);

        interrupts_enabled_ = read_register(0x00D0);
        interrupts_enabled_ |= 0x00000004;
        interrupts_enabled_ |= 0x00000080;
        interrupts_enabled_ |= 0x00000400;
        write_register(0x00D0, interrupts_enabled_);

        return true;
    }

    bool EthernetDevice::init_descriptors() {
        rx_descriptors_ = reinterpret_cast<ReceiveDescriptor*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(ReceiveDescriptor) * rx_descriptor_count_, 128));
        
        tx_descriptors_ = reinterpret_cast<TransmitDescriptor*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(TransmitDescriptor) * tx_descriptor_count_, 128));
        
        if (!rx_descriptors_ || !tx_descriptors_) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to allocate descriptor memory");
            return false;
        }

        memset(rx_descriptors_, 0, sizeof(ReceiveDescriptor) * rx_descriptor_count_);
        memset(tx_descriptors_, 0, sizeof(TransmitDescriptor) * tx_descriptor_count_);

        rx_buffers_ = new Buffer[rx_descriptor_count_];
        tx_buffers_ = new Buffer[tx_descriptor_count_];

        for (u32 i = 0; i < rx_descriptor_count_; i++) {
            rx_buffers_[i].data = reinterpret_cast<u8*>(
                mm::VirtualMemoryManager::instance().kmalloc_aligned(rx_buffer_size_, 16));
            
            if (!rx_buffers_[i].data) {
                debug::log(debug::LogLevel::Error, "ETH",
                          "Failed to allocate RX buffer %u", i);
                return false;
            }

            rx_buffers_[i].size = rx_buffer_size_;
            rx_buffers_[i].physical = mm::VirtualMemoryManager::instance()
                .get_physical_address(reinterpret_cast<virt_addr>(rx_buffers_[i].data))
                .value_or(0);
            
            if (!rx_buffers_[i].physical) {
                debug::log(debug::LogLevel::Error, "ETH",
                          "Failed to get physical address for RX buffer %u", i);
                return false;
            }

            rx_descriptors_[i].buffer_address = rx_buffers_[i].physical & 0xFFFFFFFF;
            rx_descriptors_[i].buffer_address_high = rx_buffers_[i].physical >> 32;
            rx_descriptors_[i].status = 0;
        }

        for (u32 i = 0; i < tx_descriptor_count_; i++) {
            tx_buffers_[i].data = reinterpret_cast<u8*>(
                mm::VirtualMemoryManager::instance().kmalloc_aligned(tx_buffer_size_, 16));
            
            if (!tx_buffers_[i].data) {
                debug::log(debug::LogLevel::Error, "ETH",
                          "Failed to allocate TX buffer %u", i);
                return false;
            }

            tx_buffers_[i].size = tx_buffer_size_;
            tx_buffers_[i].physical = mm::VirtualMemoryManager::instance()
                .get_physical_address(reinterpret_cast<virt_addr>(tx_buffers_[i].data))
                .value_or(0);
            
            if (!tx_buffers_[i].physical) {
                debug::log(debug::LogLevel::Error, "ETH",
                          "Failed to get physical address for TX buffer %u", i);
                return false;
            }

            tx_descriptors_[i].buffer_address = tx_buffers_[i].physical & 0xFFFFFFFF;
            tx_descriptors_[i].buffer_address_high = tx_buffers_[i].physical >> 32;
            tx_descriptors_[i].cmd = 0;
            tx_descriptors_[i].status = 0;
        }

        phys_addr rx_descriptors_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(reinterpret_cast<virt_addr>(rx_descriptors_))
            .value_or(0);
        
        phys_addr tx_descriptors_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(reinterpret_cast<virt_addr>(tx_descriptors_))
            .value_or(0);

        if (!rx_descriptors_phys || !tx_descriptors_phys) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to get physical address for descriptors");
            return false;
        }

        write_register(0x2800, rx_descriptors_phys & 0xFFFFFFFF);
        write_register(0x2804, rx_descriptors_phys >> 32);
        write_register(0x2808, rx_descriptor_count_ * sizeof(ReceiveDescriptor));

        write_register(0x3800, tx_descriptors_phys & 0xFFFFFFFF);
        write_register(0x3804, tx_descriptors_phys >> 32);
        write_register(0x3808, tx_descriptor_count_ * sizeof(TransmitDescriptor));

        write_register(0x2810, 0);
        write_register(0x3810, 0);

        u32 rctl = read_register(0x0100);
        rctl |= 0x00000001;
        write_register(0x0100, rctl);

        u32 tctl = read_register(0x0400);
        tctl |= 0x00000001;
        write_register(0x0400, tctl);

        return true;
    }

    bool EthernetDevice::start() {
        u32 rctl = read_register(0x0100);
        rctl |= 0x00000002;
        write_register(0x0100, rctl);

        u32 tctl = read_register(0x0400);
        tctl |= 0x00000002;
        write_register(0x0400, tctl);

        debug::log(debug::LogLevel::Info, "ETH", "Ethernet device started");
        return true;
    }

    bool EthernetDevice::stop() {
        u32 rctl = read_register(0x0100);
        rctl &= ~0x00000002;
        write_register(0x0100, rctl);

        u32 tctl = read_register(0x0400);
        tctl &= ~0x00000002;
        write_register(0x0400, tctl);

        debug::log(debug::LogLevel::Info, "ETH", "Ethernet device stopped");
        return true;
    }

    bool EthernetDevice::reset() {
        write_register(0x0000, 0x00000004);

        u32 timeout = 1000;
        while (timeout--) {
            if (!(read_register(0x0000) & 0x00000004)) {
                break;
            }
            arch::CPU::pause();
        }

        if (read_register(0x0000) & 0x00000004) {
            return false;
        }

        return init_hardware();
    }

    bool EthernetDevice::send(const u8* destination, u16 ether_type, const u8* data, usize size) {
        if (!destination || !data || size == 0) {
            return false;
        }

        if (size > 1514) {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Packet too large: %llu bytes", size);
            return false;
        }

        ScopedLock lock(tx_lock_);

        cleanup_tx_descriptors();

        u32 desc_index = tx_index_ % tx_descriptor_count_;
        TransmitDescriptor* desc = &tx_descriptors_[desc_index];
        Buffer* buffer = &tx_buffers_[desc_index];

        if (desc->status & 0x01) {
            debug::log(debug::LogLevel::Warning, "ETH",
                      "TX descriptor %u busy", desc_index);
            return false;
        }

        EthernetHeader header;
        memcpy(header.destination, destination, 6);
        memcpy(header.source, mac_address_, 6);
        header.ether_type = swap_endian_16(ether_type);

        memcpy(buffer->data, &header, sizeof(EthernetHeader));
        memcpy(buffer->data + sizeof(EthernetHeader), data, size);

        desc->length = sizeof(EthernetHeader) + size;
        desc->cso = 0;
        desc->cmd = 0x01 | 0x02 | 0x08 | 0x10;
        desc->status = 0;

        tx_index_++;
        write_register(0x3818, tx_index_);

        debug::log(debug::LogLevel::Trace, "ETH",
                  "Sent packet: dest=%02X:%02X:%02X:%02X:%02X:%02X, type=0x%04X, size=%llu",
                  destination[0], destination[1], destination[2],
                  destination[3], destination[4], destination[5],
                  ether_type, size);

        return true;
    }

    bool EthernetDevice::receive(u8* buffer, usize* size, u64 timeout_ms) {
        if (!buffer || !size) {
            return false;
        }

        ScopedLock lock(rx_lock_);

        u64 start_time = arch::CPU::read_tsc();
        u64 timeout_cycles = timeout_ms * 1000000;

        while (true) {
            ReceiveDescriptor* desc = &rx_descriptors_[rx_index_ % rx_descriptor_count_];
            
            if (desc->status & 0x01) {
                u16 packet_size = desc->length - 4;
                
                if (packet_size > *size) {
                    debug::log(debug::LogLevel::Warning, "ETH",
                              "Receive buffer too small: %u > %llu",
                              packet_size, *size);
                    return false;
                }

                Buffer* rx_buffer = &rx_buffers_[rx_index_ % rx_descriptor_count_];
                memcpy(buffer, rx_buffer->data, packet_size);
                *size = packet_size;

                desc->status = 0;
                update_rx_descriptor(rx_index_ % rx_descriptor_count_);

                rx_index_++;
                write_register(0x2818, rx_index_);

                return true;
            }

            if (timeout_ms != 0) {
                u64 current_time = arch::CPU::read_tsc();
                if (current_time - start_time > timeout_cycles) {
                    break;
                }
            }

            arch::CPU::pause();
        }

        return false;
    }

    bool EthernetDevice::set_mac_address(const u8* mac) {
        if (!mac) {
            return false;
        }

        memcpy(mac_address_, mac, 6);

        u32 low = (mac[3] << 24) | (mac[2] << 16) | (mac[1] << 8) | mac[0];
        u32 high = (mac[5] << 8) | mac[4];

        write_register(0x5400, low);
        write_register(0x5404, high);

        debug::log(debug::LogLevel::Info, "ETH",
                  "MAC address changed to: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        return true;
    }

    bool EthernetDevice::get_mac_address(u8* mac) {
        if (!mac) {
            return false;
        }

        u32 low = read_register(0x5400);
        u32 high = read_register(0x5404);

        mac[0] = low & 0xFF;
        mac[1] = (low >> 8) & 0xFF;
        mac[2] = (low >> 16) & 0xFF;
        mac[3] = (low >> 24) & 0xFF;
        mac[4] = high & 0xFF;
        mac[5] = (high >> 8) & 0xFF;

        return true;
    }

    bool EthernetDevice::set_promiscuous_mode(bool enable) {
        u32 rctl = read_register(0x0100);
        
        if (enable) {
            rctl |= 0x00000040;
        } else {
            rctl &= ~0x00000040;
        }
        
        write_register(0x0100, rctl);
        promiscuous_ = enable;
        
        debug::log(debug::LogLevel::Info, "ETH",
                  "Promiscuous mode %s", enable ? "enabled" : "disabled");
        
        return true;
    }

    u32 EthernetDevice::get_speed() const {
        u32 status = read_register(0x0008);
        
        if (status & 0x00000002) {
            return 1000;
        } else if (status & 0x00000020) {
            return 100;
        } else {
            return 10;
        }
    }

    bool EthernetDevice::is_link_up() const {
        u32 status = read_register(0x0008);
        return (status & 0x00000001) != 0;
    }

    bool EthernetDevice::get_statistics(u64* rx_packets, u64* tx_packets,
                                       u64* rx_bytes, u64* tx_bytes,
                                       u64* rx_errors, u64* tx_errors) {
        if (!rx_packets || !tx_packets || !rx_bytes || !tx_bytes || 
            !rx_errors || !tx_errors) {
            return false;
        }

        *rx_packets = read_register(0x04040);
        *tx_packets = read_register(0x04080);
        *rx_bytes = read_register(0x04048);
        *tx_bytes = read_register(0x04088);
        *rx_errors = read_register(0x04050);
        *tx_errors = read_register(0x04090);

        return true;
    }

    void EthernetDevice::handle_interrupt() {
        u32 interrupt_cause = read_register(0x00C0);

        if (interrupt_cause & 0x00000004) {
            debug::log(debug::LogLevel::Debug, "ETH", "Link state changed");
        }

        if (interrupt_cause & 0x00000080) {
            debug::log(debug::LogLevel::Debug, "ETH", "RX interrupt");
        }

        if (interrupt_cause & 0x00000400) {
            debug::log(debug::LogLevel::Debug, "ETH", "TX interrupt");
        }

        write_register(0x00C0, interrupt_cause);
    }

    void EthernetDevice::poll() {
        handle_interrupt();
    }

    void EthernetDevice::dump_registers() const {
        debug::log(debug::LogLevel::Info, "ETH", "Device Registers:");
        debug::log(debug::LogLevel::Info, "ETH", "  CTRL: 0x%08X", read_register(0x0000));
        debug::log(debug::LogLevel::Info, "ETH", "  STATUS: 0x%08X", read_register(0x0008));
        debug::log(debug::LogLevel::Info, "ETH", "  RCTL: 0x%08X", read_register(0x0100));
        debug::log(debug::LogLevel::Info, "ETH", "  TCTL: 0x%08X", read_register(0x0400));
        debug::log(debug::LogLevel::Info, "ETH", "  RDBAL: 0x%08X", read_register(0x2800));
        debug::log(debug::LogLevel::Info, "ETH", "  RDBAH: 0x%08X", read_register(0x2804));
        debug::log(debug::LogLevel::Info, "ETH", "  RDLEN: 0x%08X", read_register(0x2808));
        debug::log(debug::LogLevel::Info, "ETH", "  RDH: 0x%08X", read_register(0x2810));
        debug::log(debug::LogLevel::Info, "ETH", "  RDT: 0x%08X", read_register(0x2818));
        debug::log(debug::LogLevel::Info, "ETH", "  TDBAL: 0x%08X", read_register(0x3800));
        debug::log(debug::LogLevel::Info, "ETH", "  TDBAH: 0x%08X", read_register(0x3804));
        debug::log(debug::LogLevel::Info, "ETH", "  TDLEN: 0x%08X", read_register(0x3808));
        debug::log(debug::LogLevel::Info, "ETH", "  TDH: 0x%08X", read_register(0x3810));
        debug::log(debug::LogLevel::Info, "ETH", "  TDT: 0x%08X", read_register(0x3818));
    }

    void EthernetDevice::dump_statistics() const {
        u64 rx_packets, tx_packets, rx_bytes, tx_bytes, rx_errors, tx_errors;
        
        if (get_statistics(&rx_packets, &tx_packets, &rx_bytes, &tx_bytes, 
                          &rx_errors, &tx_errors)) {
            debug::log(debug::LogLevel::Info, "ETH", "Device Statistics:");
            debug::log(debug::LogLevel::Info, "ETH", "  RX packets: %llu", rx_packets);
            debug::log(debug::LogLevel::Info, "ETH", "  TX packets: %llu", tx_packets);
            debug::log(debug::LogLevel::Info, "ETH", "  RX bytes: %llu", rx_bytes);
            debug::log(debug::LogLevel::Info, "ETH", "  TX bytes: %llu", tx_bytes);
            debug::log(debug::LogLevel::Info, "ETH", "  RX errors: %llu", rx_errors);
            debug::log(debug::LogLevel::Info, "ETH", "  TX errors: %llu", tx_errors);
        }
    }

    void EthernetDevice::update_rx_descriptor(u32 index) {
        ReceiveDescriptor* desc = &rx_descriptors_[index];
        desc->status = 0;
    }

    void EthernetDevice::update_tx_descriptor(u32 index) {
        TransmitDescriptor* desc = &tx_descriptors_[index];
        desc->cmd = 0;
        desc->status = 0;
    }

    void EthernetDevice::cleanup_tx_descriptors() {
        while (tx_clean_index_ != tx_index_) {
            u32 desc_index = tx_clean_index_ % tx_descriptor_count_;
            TransmitDescriptor* desc = &tx_descriptors_[desc_index];
            
            if (!(desc->status & 0x01)) {
                break;
            }
            
            update_tx_descriptor(desc_index);
            tx_clean_index_++;
        }
    }

    u32 EthernetDevice::read_register(u32 offset) {
        if (io_base_) {
            return arch::inl(reinterpret_cast<u16>(io_base_ + offset));
        } else if (memory_base_) {
            volatile u32* reg = reinterpret_cast<volatile u32*>(memory_base_ + offset);
            return *reg;
        }
        return 0;
    }

    void EthernetDevice::write_register(u32 offset, u32 value) {
        if (io_base_) {
            arch::outl(reinterpret_cast<u16>(io_base_ + offset), value);
        } else if (memory_base_) {
            volatile u32* reg = reinterpret_cast<volatile u32*>(memory_base_ + offset);
            *reg = value;
        }
    }

    EthernetManager::EthernetManager() {
        debug::log(debug::LogLevel::Info, "ETH", "Ethernet Manager created");
    }

    EthernetManager::~EthernetManager() {
        ScopedLock lock(lock_);
        
        for (EthernetDevice* device : devices_) {
            delete device;
        }
        devices_.clear();
    }

    bool EthernetManager::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "ETH", "Initializing Ethernet Manager");
        
        Vector<PCI::Device*> network_devices = PCI::find_devices_by_class(0x02, 0x00, 0x00);
        
        for (PCI::Device* device : network_devices) {
            debug::log(debug::LogLevel::Info, "ETH",
                      "Found network device at %02X:%02X.%X",
                      device->get_bus(), device->get_slot(), device->get_function());
            
            add_device(device);
        }
        
        debug::log(debug::LogLevel::Info, "ETH", 
                  "Ethernet Manager initialized with %llu devices",
                  devices_.size());
        
        return true;
    }

    bool EthernetManager::add_device(PCI::Device* pci_device) {
        ScopedLock lock(lock_);
        
        EthernetDevice* device = new EthernetDevice(pci_device);
        if (device->init()) {
            devices_.push_back(device);
            debug::log(debug::LogLevel::Success, "ETH",
                      "Added Ethernet device");
            device->dump_registers();
            return true;
        } else {
            debug::log(debug::LogLevel::Error, "ETH",
                      "Failed to initialize Ethernet device");
            delete device;
            return false;
        }
    }

    EthernetDevice* EthernetManager::get_device(u32 index) {
        ScopedLock lock(lock_);
        
        if (index >= devices_.size()) {
            return nullptr;
        }
        
        return devices_[index];
    }

    bool EthernetManager::send(u32 device_index, const u8* destination, u16 ether_type,
                              const u8* data, usize size) {
        EthernetDevice* device = get_device(device_index);
        if (!device) {
            return false;
        }
        
        return device->send(destination, ether_type, data, size);
    }

    bool EthernetManager::broadcast(u32 device_index, u16 ether_type,
                                   const u8* data, usize size) {
        u8 broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        return send(device_index, broadcast_addr, ether_type, data, size);
    }

    bool EthernetManager::register_callback(u16 ether_type, ReceiveCallback callback, void* user_data) {
        ScopedLock lock(callback_lock_);
        
        CallbackEntry entry;
        entry.ether_type = ether_type;
        entry.callback = callback;
        entry.user_data = user_data;
        
        callbacks_.push_back(entry);
        
        debug::log(debug::LogLevel::Info, "ETH",
                  "Registered callback for ethertype 0x%04X", ether_type);
        
        return true;
    }

    bool EthernetManager::unregister_callback(u16 ether_type, ReceiveCallback callback) {
        ScopedLock lock(callback_lock_);
        
        for (usize i = 0; i < callbacks_.size(); i++) {
            if (callbacks_[i].ether_type == ether_type && 
                callbacks_[i].callback == callback) {
                callbacks_.erase(i);
                return true;
            }
        }
        
        return false;
    }

    void EthernetManager::process_packets() {
        ScopedLock lock(lock_);
        
        for (EthernetDevice* device : devices_) {
            u8 buffer[2048];
            usize size = sizeof(buffer);
            
            if (device->receive(buffer, &size, 0)) {
                if (size < sizeof(EthernetHeader)) {
                    continue;
                }
                
                EthernetHeader* header = reinterpret_cast<EthernetHeader*>(buffer);
                u16 ether_type = swap_endian_16(header->ether_type);
                
                ScopedLock callback_lock(callback_lock_);
                for (const CallbackEntry& entry : callbacks_) {
                    if (entry.ether_type == ether_type) {
                        entry.callback(header->source, header->destination,
                                      ether_type, buffer + sizeof(EthernetHeader),
                                      size - sizeof(EthernetHeader), entry.user_data);
                    }
                }
            }
        }
    }

    void EthernetManager::poll_devices() {
        ScopedLock lock(lock_);
        
        for (EthernetDevice* device : devices_) {
            device->poll();
        }
        
        process_packets();
    }

    void EthernetManager::dump_devices() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "ETH", "Ethernet Devices:");
        for (usize i = 0; i < devices_.size(); i++) {
            const u8* mac = devices_[i]->get_mac_address();
            debug::log(debug::LogLevel::Info, "ETH",
                      "  Device %llu: MAC=%02X:%02X:%02X:%02X:%02X:%02X, Link=%s, Speed=%u Mbps",
                      i, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                      devices_[i]->is_link_up() ? "up" : "down",
                      devices_[i]->get_speed());
        }
    }

    EthernetManager& EthernetManager::instance() {
        static EthernetManager instance;
        return instance;
    }
}
