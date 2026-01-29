#include <nanokoton/drivers/ahci.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/bitops.hpp>
#include <nanokoton/arch/io.hpp>

namespace nk::drivers {
    Vector<AHCIController*> AHCIManager::controllers_;
    SpinLock AHCIManager::lock_;

    AHCIController::AHCIController(PCI::Device* pci_device)
        : pci_device_(pci_device),
          hba_(nullptr),
          capabilities_(0),
          ports_implemented_(0),
          version_(0) {
        if (pci_device_) {
            pci_device_->enable_bus_mastering();
            pci_device_->enable_memory_space();
        }
    }

    AHCIController::~AHCIController() {
        if (hba_) {
            mm::VirtualMemoryManager::instance().kfree(hba_);
        }
    }

    bool AHCIController::init() {
        if (!pci_device_) {
            return false;
        }

        if (!find_device()) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "Failed to find AHCI controller");
            return false;
        }

        if (!init_hba()) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "Failed to initialize HBA");
            return false;
        }

        for (u32 i = 0; i < 32; i++) {
            if (ports_implemented_ & (1 << i)) {
                if (probe_port(i)) {
                    debug::log(debug::LogLevel::Info, "AHCI",
                              "Port %u initialized successfully", i);
                }
            }
        }

        debug::log(debug::LogLevel::Info, "AHCI", 
                  "AHCI controller initialized with %llu ports", ports_.size());
        
        return true;
    }

    bool AHCIController::find_device() {
        if (!pci_device_->check_vendor_id(0x8086) && 
            !pci_device_->check_vendor_id(0x1002) &&
            !pci_device_->check_vendor_id(0x10DE)) {
            return false;
        }

        if (pci_device_->get_class_code() != 0x01 || 
            pci_device_->get_subclass() != 0x06) {
            return false;
        }

        return true;
    }

    bool AHCIController::init_hba() {
        u64 hba_phys = pci_device_->get_bar(5) & ~0xF;
        if (!hba_phys) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "No valid HBA BAR found");
            return false;
        }

        hba_ = reinterpret_cast<AHCIHostControl*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(AHCIHostControl), 4096));
        
        if (!hba_) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "Failed to allocate memory for HBA");
            return false;
        }

        if (!mm::VirtualMemoryManager::instance().map_page(
                reinterpret_cast<virt_addr>(hba_), hba_phys,
                mm::PageFlags::Present | mm::PageFlags::Writable | 
                mm::PageFlags::CacheDisabled)) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "Failed to map HBA memory");
            return false;
        }

        capabilities_ = hba_->capabilities;
        ports_implemented_ = hba_->ports_implemented;
        version_ = hba_->version;

        hba_->global_host_control |= (1 << 31);

        u32 timeout = 1000;
        while (timeout--) {
            if (hba_->global_host_control & (1 << 31)) {
                break;
            }
            asm volatile("pause");
        }

        if (!(hba_->global_host_control & (1 << 31))) {
            debug::log(debug::LogLevel::Error, "AHCI", 
                      "HBA failed to start");
            return false;
        }

        hba_->global_host_control |= (1 << 1);

        debug::log(debug::LogLevel::Debug, "AHCI",
                  "HBA initialized: caps=0x%08X, ports=0x%08X, version=0x%08X",
                  capabilities_, ports_implemented_, version_);
        
        return true;
    }

    bool AHCIController::probe_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        u32 sata_status = port.sata_status;
        u32 interface_power_management = (sata_status >> 8) & 0x0F;
        u32 device_detection = sata_status & 0x0F;

        if (device_detection != 3) {
            debug::log(debug::LogLevel::Debug, "AHCI",
                      "Port %u: No device detected (status=0x%08X)", 
                      port_number, sata_status);
            return false;
        }

        if (interface_power_management != 1) {
            debug::log(debug::LogLevel::Warning, "AHCI",
                      "Port %u: Device not in active state", port_number);
        }

        u32 signature = port.signature;
        
        PortInfo info;
        info.number = port_number;
        info.initialized = false;

        switch (signature) {
            case 0xEB140101:
                info.type = 1;
                debug::log(debug::LogLevel::Info, "AHCI",
                          "Port %u: SATAPI device", port_number);
                break;
            case 0xC33C0101:
                info.type = 2;
                debug::log(debug::LogLevel::Info, "AHCI",
                          "Port %u: Enclosure management bridge", port_number);
                break;
            case 0x96690101:
                info.type = 3;
                debug::log(debug::LogLevel::Info, "AHCI",
                          "Port %u: Port multiplier", port_number);
                break;
            default:
                info.type = 0;
                debug::log(debug::LogLevel::Info, "AHCI",
                          "Port %u: SATA device (signature=0x%08X)", 
                          port_number, signature);
                break;
        }

        if (!reset_port(port_number)) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to reset", port_number);
            return false;
        }

        if (!start_port(port_number)) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to start", port_number);
            return false;
        }

        if (!identify_device(port_number, info)) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to identify device", port_number);
            return false;
        }

        info.initialized = true;
        ports_.push_back(info);
        
        return true;
    }

    bool AHCIController::reset_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        port.command_status &= ~0x01;

        u32 timeout = 1000;
        while (timeout--) {
            if (!(port.command_status & 0x8000)) {
                break;
            }
            asm volatile("pause");
        }

        if (port.command_status & 0x8000) {
            return false;
        }

        port.sata_control |= 0x01;

        timeout = 1000;
        while (timeout--) {
            if (!(port.sata_control & 0x01)) {
                break;
            }
            asm volatile("pause");
        }

        if (port.sata_control & 0x01) {
            return false;
        }

        return true;
    }

    bool AHCIController::start_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        port.command_status |= 0x01;

        u32 timeout = 1000;
        while (timeout--) {
            if (port.command_status & 0x8000) {
                break;
            }
            asm volatile("pause");
        }

        if (!(port.command_status & 0x8000)) {
            return false;
        }

        return true;
    }

    bool AHCIController::stop_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        port.command_status &= ~0x01;

        u32 timeout = 1000;
        while (timeout--) {
            if (!(port.command_status & 0x8000)) {
                break;
            }
            asm volatile("pause");
        }

        return true;
    }

    bool AHCIController::identify_device(u32 port_number, PortInfo& info) {
        u8* identify_data = reinterpret_cast<u8*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(512, 512));
        
        if (!identify_data) {
            return false;
        }

        if (!read_sectors(port_number, 0, 1, identify_data)) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            return false;
        }

        u16* identify_words = reinterpret_cast<u16*>(identify_data);

        info.sector_size = 512;
        
        if (identify_words[106] & (1 << 14)) {
            info.supports_48bit = true;
        } else {
            info.supports_48bit = false;
        }

        if (identify_words[76] & (1 << 8)) {
            info.supports_ncq = true;
        } else {
            info.supports_ncq = false;
        }

        if (info.supports_48bit) {
            info.sector_count = 
                (static_cast<u64>(identify_words[100]) << 0) |
                (static_cast<u64>(identify_words[101]) << 16) |
                (static_cast<u64>(identify_words[102]) << 32) |
                (static_cast<u64>(identify_words[103]) << 48);
        } else {
            info.sector_count = 
                (static_cast<u64>(identify_words[61]) << 0) |
                (static_cast<u64>(identify_words[60]) << 16);
        }

        for (usize i = 0; i < 20; i++) {
            info.model[i * 2] = identify_data[54 + i * 2 + 1];
            info.model[i * 2 + 1] = identify_data[54 + i * 2];
        }
        info.model[40] = '\0';

        for (usize i = 0; i < 10; i++) {
            info.serial[i * 2] = identify_data[20 + i * 2 + 1];
            info.serial[i * 2 + 1] = identify_data[20 + i * 2];
        }
        info.serial[20] = '\0';

        for (usize i = 0; i < 4; i++) {
            info.firmware[i * 2] = identify_data[46 + i * 2 + 1];
            info.firmware[i * 2 + 1] = identify_data[46 + i * 2];
        }
        info.firmware[8] = '\0';

        debug::log(debug::LogLevel::Info, "AHCI",
                  "Port %u: Model='%s', Serial='%s', Firmware='%s'",
                  port_number, info.model, info.serial, info.firmware);
        
        debug::log(debug::LogLevel::Info, "AHCI",
                  "Port %u: Sectors=%llu, Size=%llu MB, 48-bit=%s, NCQ=%s",
                  port_number, info.sector_count,
                  (info.sector_count * info.sector_size) / (1024 * 1024),
                  info.supports_48bit ? "yes" : "no",
                  info.supports_ncq ? "yes" : "no");

        mm::VirtualMemoryManager::instance().kfree(identify_data);
        return true;
    }

    bool AHCIController::wait_for_clear(u32 port_number, u32 offset, u32 mask, u32 timeout) {
        AHCIPort& port = hba_->ports[port_number];
        u32* reg = reinterpret_cast<u32*>(reinterpret_cast<u8*>(&port) + offset);
        
        while (timeout--) {
            if ((*reg & mask) == 0) {
                return true;
            }
            asm volatile("pause");
        }
        
        return false;
    }

    bool AHCIController::wait_for_set(u32 port_number, u32 offset, u32 mask, u32 timeout) {
        AHCIPort& port = hba_->ports[port_number];
        u32* reg = reinterpret_cast<u32*>(reinterpret_cast<u8*>(&port) + offset);
        
        while (timeout--) {
            if ((*reg & mask) == mask) {
                return true;
            }
            asm volatile("pause");
        }
        
        return false;
    }

    bool AHCIController::send_command(u32 port_number, HBACommandHeader* header,
                                      HBACommandTable* table, void* buffer, usize buffer_size) {
        AHCIPort& port = hba_->ports[port_number];
        
        port.interrupt_status = 0xFFFFFFFF;
        
        u32 slot = 0;
        
        port.command_issue = 1 << slot;
        
        if (!wait_for_clear(port_number, offsetof(AHCIPort, command_issue), 
                           1 << slot, 10000)) {
            return false;
        }
        
        if (port.interrupt_status & (1 << 30)) {
            port.interrupt_status = 1 << 30;
            return false;
        }
        
        return true;
    }

    bool AHCIController::read_sectors(u32 port_number, u64 lba, u32 count, void* buffer) {
        if (port_number >= ports_.size()) {
            return false;
        }

        const PortInfo& info = ports_[port_number];
        
        if (!info.initialized) {
            return false;
        }

        if (lba + count > info.sector_count) {
            return false;
        }

        HBACommandHeader* header = reinterpret_cast<HBACommandHeader*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(HBACommandHeader), 256));
        
        if (!header) {
            return false;
        }

        HBACommandTable* table = reinterpret_cast<HBACommandTable*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(HBACommandTable) + sizeof(HBACommandTable::HBAPRDTEntry), 256));
        
        if (!table) {
            mm::VirtualMemoryManager::instance().kfree(header);
            return false;
        }

        memset(header, 0, sizeof(HBACommandHeader));
        memset(table, 0, sizeof(HBACommandTable) + sizeof(HBACommandTable::HBAPRDTEntry));

        FISRegisterH2D* fis = reinterpret_cast<FISRegisterH2D*>(table->command_fis);
        fis->fis_type = 0x27;
        fis->command_control = 1;
        fis->command = info.supports_48bit ? 0x25 : 0x20;
        
        if (info.supports_48bit) {
            fis->lba0 = (lba >> 0) & 0xFF;
            fis->lba1 = (lba >> 8) & 0xFF;
            fis->lba2 = (lba >> 16) & 0xFF;
            fis->lba3 = (lba >> 24) & 0xFF;
            fis->lba4 = (lba >> 32) & 0xFF;
            fis->lba5 = (lba >> 40) & 0xFF;
            fis->count_low = count & 0xFF;
            fis->count_high = (count >> 8) & 0xFF;
        } else {
            fis->lba0 = (lba >> 0) & 0xFF;
            fis->lba1 = (lba >> 8) & 0xFF;
            fis->lba2 = (lba >> 16) & 0xFF;
            fis->device = 0x40 | ((lba >> 24) & 0x0F);
            fis->count_low = count & 0xFF;
        }

        fis->device |= 0xE0;

        header->command_fis_length = sizeof(FISRegisterH2D) / sizeof(u32);
        header->write = 0;
        header->prdt_length = 1;

        virt_addr buffer_virt = reinterpret_cast<virt_addr>(buffer);
        phys_addr buffer_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(buffer_virt).value_or(0);
        
        if (!buffer_phys) {
            mm::VirtualMemoryManager::instance().kfree(table);
            mm::VirtualMemoryManager::instance().kfree(header);
            return false;
        }

        table->prdt_entries[0].data_base_address = buffer_phys & 0xFFFFFFFF;
        table->prdt_entries[0].data_base_address_upper = buffer_phys >> 32;
        table->prdt_entries[0].byte_count = (count * info.sector_size) - 1;
        table->prdt_entries[0].interrupt_on_completion = 1;

        virt_addr header_virt = reinterpret_cast<virt_addr>(header);
        phys_addr header_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(header_virt).value_or(0);
        
        virt_addr table_virt = reinterpret_cast<virt_addr>(table);
        phys_addr table_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(table_virt).value_or(0);

        header->command_table_base_address = table_phys & 0xFFFFFFFF;
        header->command_table_base_address_upper = table_phys >> 32;

        AHCIPort& port = hba_->ports[info.number];
        phys_addr clb_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(reinterpret_cast<virt_addr>(header)).value_or(0);
        
        port.command_list_base = clb_phys;
        port.fis_base = 0;

        bool success = send_command(port_number, header, table, buffer, count * info.sector_size);

        mm::VirtualMemoryManager::instance().kfree(table);
        mm::VirtualMemoryManager::instance().kfree(header);

        return success;
    }

    bool AHCIController::write_sectors(u32 port_number, u64 lba, u32 count, const void* buffer) {
        if (port_number >= ports_.size()) {
            return false;
        }

        const PortInfo& info = ports_[port_number];
        
        if (!info.initialized) {
            return false;
        }

        if (lba + count > info.sector_count) {
            return false;
        }

        HBACommandHeader* header = reinterpret_cast<HBACommandHeader*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(HBACommandHeader), 256));
        
        if (!header) {
            return false;
        }

        HBACommandTable* table = reinterpret_cast<HBACommandTable*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(HBACommandTable) + sizeof(HBACommandTable::HBAPRDTEntry), 256));
        
        if (!table) {
            mm::VirtualMemoryManager::instance().kfree(header);
            return false;
        }

        memset(header, 0, sizeof(HBACommandHeader));
        memset(table, 0, sizeof(HBACommandTable) + sizeof(HBACommandTable::HBAPRDTEntry));

        FISRegisterH2D* fis = reinterpret_cast<FISRegisterH2D*>(table->command_fis);
        fis->fis_type = 0x27;
        fis->command_control = 1;
        fis->command = info.supports_48bit ? 0x35 : 0x30;
        
        if (info.supports_48bit) {
            fis->lba0 = (lba >> 0) & 0xFF;
            fis->lba1 = (lba >> 8) & 0xFF;
            fis->lba2 = (lba >> 16) & 0xFF;
            fis->lba3 = (lba >> 24) & 0xFF;
            fis->lba4 = (lba >> 32) & 0xFF;
            fis->lba5 = (lba >> 40) & 0xFF;
            fis->count_low = count & 0xFF;
            fis->count_high = (count >> 8) & 0xFF;
        } else {
            fis->lba0 = (lba >> 0) & 0xFF;
            fis->lba1 = (lba >> 8) & 0xFF;
            fis->lba2 = (lba >> 16) & 0xFF;
            fis->device = 0x40 | ((lba >> 24) & 0x0F);
            fis->count_low = count & 0xFF;
        }

        fis->device |= 0xE0;

        header->command_fis_length = sizeof(FISRegisterH2D) / sizeof(u32);
        header->write = 1;
        header->prdt_length = 1;

        virt_addr buffer_virt = reinterpret_cast<virt_addr>(buffer);
        phys_addr buffer_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(buffer_virt).value_or(0);
        
        if (!buffer_phys) {
            mm::VirtualMemoryManager::instance().kfree(table);
            mm::VirtualMemoryManager::instance().kfree(header);
            return false;
        }

        table->prdt_entries[0].data_base_address = buffer_phys & 0xFFFFFFFF;
        table->prdt_entries[0].data_base
