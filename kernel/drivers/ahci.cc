#include <nanokoton/drivers/ahci.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/mm/physical.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/bitops.hpp>
#include <nanokoton/lib/algorithm.hpp>
#include <nanokoton/arch/io.hpp>
#include <nanokoton/arch/cpu.hpp>

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
            pci_device_->enable_io_space();
        }
    }

    AHCIController::~AHCIController() {
        if (hba_) {
            for (usize i = 0; i < 32; i++) {
                if (ports_implemented_ & (1 << i)) {
                    stop_port(i);
                }
            }
            
            mm::VirtualMemoryManager::instance().kfree(hba_);
        }
    }

    bool AHCIController::init() {
        if (!pci_device_) {
            debug::log(debug::LogLevel::Error, "AHCI", "No PCI device provided");
            return false;
        }

        if (!find_device()) {
            debug::log(debug::LogLevel::Error, "AHCI", "Not a valid AHCI controller");
            return false;
        }

        if (!init_hba()) {
            debug::log(debug::LogLevel::Error, "AHCI", "Failed to initialize HBA");
            return false;
        }

        u32 successful_ports = 0;
        for (u32 i = 0; i < 32; i++) {
            if (ports_implemented_ & (1 << i)) {
                if (probe_port(i)) {
                    successful_ports++;
                }
            }
        }

        debug::log(debug::LogLevel::Info, "AHCI", 
                  "AHCI controller initialized with %u/%u ports active",
                  successful_ports, count_bits(ports_implemented_));
        
        return successful_ports > 0;
    }

    bool AHCIController::find_device() {
        u32 class_code = pci_device_->get_class_code();
        u32 subclass = pci_device_->get_subclass();
        u32 prog_if = pci_device_->get_prog_if();

        debug::log(debug::LogLevel::Debug, "AHCI",
                  "PCI device: class=0x%02X, subclass=0x%02X, prog_if=0x%02X",
                  class_code, subclass, prog_if);

        return class_code == 0x01 && subclass == 0x06 && prog_if == 0x01;
    }

    bool AHCIController::init_hba() {
        u64 hba_phys = pci_device_->get_bar(5);
        
        if ((hba_phys & 1) == 0) {
            hba_phys &= ~0xF;
        } else {
            hba_phys &= ~0x3;
        }

        if (!hba_phys) {
            debug::log(debug::LogLevel::Error, "AHCI", "Invalid HBA BAR");
            return false;
        }

        debug::log(debug::LogLevel::Debug, "AHCI", "HBA physical address: 0x%016llX", hba_phys);

        hba_ = reinterpret_cast<AHCIHostControl*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(
                sizeof(AHCIHostControl), 4096));
        
        if (!hba_) {
            debug::log(debug::LogLevel::Error, "AHCI", "Failed to allocate memory for HBA");
            return false;
        }

        memset(hba_, 0, sizeof(AHCIHostControl));

        if (!mm::VirtualMemoryManager::instance().map_page(
                reinterpret_cast<virt_addr>(hba_), hba_phys,
                mm::PageFlags::Present | mm::PageFlags::Writable | 
                mm::PageFlags::CacheDisabled)) {
            debug::log(debug::LogLevel::Error, "AHCI", "Failed to map HBA memory");
            mm::VirtualMemoryManager::instance().kfree(hba_);
            hba_ = nullptr;
            return false;
        }

        capabilities_ = hba_->capabilities;
        ports_implemented_ = hba_->ports_implemented;
        version_ = hba_->version;

        debug::log(debug::LogLevel::Debug, "AHCI",
                  "HBA capabilities: 0x%08X, ports: 0x%08X, version: 0x%08X",
                  capabilities_, ports_implemented_, version_);

        if ((capabilities_ & (1 << 31)) == 0) {
            debug::log(debug::LogLevel::Warning, "AHCI", "HBA does not support 64-bit addressing");
        }

        u32 ghc = hba_->global_host_control;
        if ((ghc & (1 << 31)) == 0) {
            hba_->global_host_control = ghc | (1 << 31);

            u32 timeout = 1000000;
            while (timeout--) {
                if (hba_->global_host_control & (1 << 31)) {
                    break;
                }
                arch::CPU::pause();
            }

            if ((hba_->global_host_control & (1 << 31)) == 0) {
                debug::log(debug::LogLevel::Error, "AHCI", "Failed to start HBA");
                return false;
            }
        }

        hba_->global_host_control |= (1 << 1);

        if (hba_->bios_handoff_control_status & (1 << 0)) {
            hba_->bios_handoff_control_status |= (1 << 1);
            
            u32 timeout = 25000;
            while (timeout--) {
                if ((hba_->bios_handoff_control_status & (1 << 0)) == 0) {
                    break;
                }
                arch::CPU::pause();
            }
            
            if (hba_->bios_handoff_control_status & (1 << 0)) {
                debug::log(debug::LogLevel::Warning, "AHCI", "BIOS handoff timeout");
            }
        }

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

        debug::log(debug::LogLevel::Debug, "AHCI",
                  "Port %u: SATA status=0x%08X, IPM=%u, DET=%u",
                  port_number, sata_status, interface_power_management, device_detection);

        if (device_detection != 3) {
            debug::log(debug::LogLevel::Warning, "AHCI",
                      "Port %u: No device detected", port_number);
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
            stop_port(port_number);
            return false;
        }

        info.initialized = true;
        
        {
            ScopedLock lock(lock_);
            ports_.push_back(info);
        }
        
        debug::log(debug::LogLevel::Success, "AHCI",
                  "Port %u: Device '%s' initialized, %llu sectors",
                  port_number, info.model, info.sector_count);
        
        return true;
    }

    bool AHCIController::reset_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        port.command_status &= ~0x01;

        u32 timeout = 1000000;
        while (timeout--) {
            if ((port.command_status & 0x8000) == 0) {
                break;
            }
            arch::CPU::pause();
        }

        if (port.command_status & 0x8000) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to clear command running", port_number);
            return false;
        }

        port.sata_control |= 0x01;

        timeout = 1000000;
        while (timeout--) {
            if ((port.sata_control & 0x01) == 0) {
                break;
            }
            arch::CPU::pause();
        }

        if (port.sata_control & 0x01) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to reset", port_number);
            return false;
        }

        return true;
    }

    bool AHCIController::start_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        auto page_phys_opt = mm::PhysicalMemoryManager::instance().allocate_pages(2);
        if (!page_phys_opt.has_value()) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to allocate command list", port_number);
            return false;
        }

        phys_addr cl_phys = page_phys_opt.value();
        virt_addr cl_virt = mm::VirtualMemoryManager::instance().kmalloc_aligned(8192, 8192);
        
        if (!cl_virt) {
            mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to allocate virtual memory for command list", port_number);
            return false;
        }

        if (!mm::VirtualMemoryManager::instance().map_pages(cl_virt, cl_phys, 2,
                mm::PageFlags::Present | mm::PageFlags::Writable | mm::PageFlags::CacheDisabled)) {
            mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
            mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to map command list", port_number);
            return false;
        }

        HBACommandHeader* cmd_list = reinterpret_cast<HBACommandHeader*>(cl_virt);
        memset(cmd_list, 0, 1024);

        auto fis_phys_opt = mm::PhysicalMemoryManager::instance().allocate_page();
        if (!fis_phys_opt.has_value()) {
            mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
            mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to allocate FIS", port_number);
            return false;
        }

        phys_addr fis_phys = fis_phys_opt.value();
        virt_addr fis_virt = mm::VirtualMemoryManager::instance().kmalloc_aligned(4096, 4096);
        
        if (!fis_virt) {
            mm::PhysicalMemoryManager::instance().free_page(fis_phys);
            mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
            mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to allocate virtual memory for FIS", port_number);
            return false;
        }

        if (!mm::VirtualMemoryManager::instance().map_page(fis_virt, fis_phys,
                mm::PageFlags::Present | mm::PageFlags::Writable | mm::PageFlags::CacheDisabled)) {
            mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(fis_virt));
            mm::PhysicalMemoryManager::instance().free_page(fis_phys);
            mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
            mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to map FIS", port_number);
            return false;
        }

        u8* fis_base = reinterpret_cast<u8*>(fis_virt);
        memset(fis_base, 0, 256);

        for (u32 i = 0; i < 32; i++) {
            auto table_phys_opt = mm::PhysicalMemoryManager::instance().allocate_page();
            if (!table_phys_opt.has_value()) {
                for (u32 j = 0; j < i; j++) {
                    phys_addr prev_phys = cmd_list[j].command_table_base_address | 
                                          (static_cast<u64>(cmd_list[j].command_table_base_address_upper) << 32);
                    mm::PhysicalMemoryManager::instance().free_page(prev_phys);
                }
                
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(fis_virt));
                mm::PhysicalMemoryManager::instance().free_page(fis_phys);
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
                mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
                
                debug::log(debug::LogLevel::Error, "AHCI",
                          "Port %u: Failed to allocate command table %u", port_number, i);
                return false;
            }

            phys_addr table_phys = table_phys_opt.value();
            virt_addr table_virt = mm::VirtualMemoryManager::instance().kmalloc_aligned(4096, 4096);
            
            if (!table_virt) {
                mm::PhysicalMemoryManager::instance().free_page(table_phys);
                for (u32 j = 0; j < i; j++) {
                    phys_addr prev_phys = cmd_list[j].command_table_base_address | 
                                          (static_cast<u64>(cmd_list[j].command_table_base_address_upper) << 32);
                    mm::PhysicalMemoryManager::instance().free_page(prev_phys);
                }
                
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(fis_virt));
                mm::PhysicalMemoryManager::instance().free_page(fis_phys);
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
                mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
                
                debug::log(debug::LogLevel::Error, "AHCI",
                          "Port %u: Failed to allocate virtual memory for command table %u", 
                          port_number, i);
                return false;
            }

            if (!mm::VirtualMemoryManager::instance().map_page(table_virt, table_phys,
                    mm::PageFlags::Present | mm::PageFlags::Writable | mm::PageFlags::CacheDisabled)) {
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(table_virt));
                mm::PhysicalMemoryManager::instance().free_page(table_phys);
                for (u32 j = 0; j < i; j++) {
                    phys_addr prev_phys = cmd_list[j].command_table_base_address | 
                                          (static_cast<u64>(cmd_list[j].command_table_base_address_upper) << 32);
                    mm::PhysicalMemoryManager::instance().free_page(prev_phys);
                }
                
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(fis_virt));
                mm::PhysicalMemoryManager::instance().free_page(fis_phys);
                mm::VirtualMemoryManager::instance().kfree(reinterpret_cast<void*>(cl_virt));
                mm::PhysicalMemoryManager::instance().free_pages(cl_phys, 2);
                
                debug::log(debug::LogLevel::Error, "AHCI",
                          "Port %u: Failed to map command table %u", port_number, i);
                return false;
            }

            HBACommandTable* table = reinterpret_cast<HBACommandTable*>(table_virt);
            memset(table, 0, sizeof(HBACommandTable) + 8 * sizeof(HBACommandTable::HBAPRDTEntry));

            cmd_list[i].command_table_base_address = table_phys & 0xFFFFFFFF;
            cmd_list[i].command_table_base_address_upper = table_phys >> 32;
            cmd_list[i].prdt_length = 8;
        }

        port.command_list_base = cl_phys & 0xFFFFFFFF;
        port.command_list_base_upper = cl_phys >> 32;
        port.fis_base = fis_phys & 0xFFFFFFFF;
        port.fis_base_upper = fis_phys >> 32;

        port.interrupt_status = 0xFFFFFFFF;
        port.interrupt_enable = 0;

        u32 cmd = port.command_status;
        cmd |= 0x01;
        cmd &= ~0x02;
        port.command_status = cmd;

        u32 timeout = 1000000;
        while (timeout--) {
            if (port.command_status & 0x8000) {
                break;
            }
            arch::CPU::pause();
        }

        if (!(port.command_status & 0x8000)) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to start command engine", port_number);
            
            port.command_list_base = 0;
            port.command_list_base_upper = 0;
            port.fis_base = 0;
            port.fis_base_upper = 0;
            
            return false;
        }

        port.interrupt_enable = 0xFFFFFFFF;

        return true;
    }

    bool AHCIController::stop_port(u32 port_number) {
        if (port_number >= 32) {
            return false;
        }

        AHCIPort& port = hba_->ports[port_number];

        port.interrupt_enable = 0;

        u32 cmd = port.command_status;
        cmd &= ~0x01;
        port.command_status = cmd;

        u32 timeout = 1000000;
        while (timeout--) {
            if ((port.command_status & 0x8000) == 0) {
                break;
            }
            arch::CPU::pause();
        }

        if (port.command_status & 0x8000) {
            debug::log(debug::LogLevel::Warning, "AHCI",
                      "Port %u: Command engine still running after stop", port_number);
        }

        port.interrupt_status = 0xFFFFFFFF;

        return true;
    }

    bool AHCIController::identify_device(u32 port_number, PortInfo& info) {
        u8* identify_data = reinterpret_cast<u8*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(512, 512));
        
        if (!identify_data) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to allocate memory for IDENTIFY", port_number);
            return false;
        }

        memset(identify_data, 0, 512);

        HBACommandHeader* header = reinterpret_cast<HBACommandHeader*>(
            reinterpret_cast<virt_addr>(hba_->ports[port_number].command_list_base) |
            (static_cast<u64>(hba_->ports[port_number].command_list_base_upper) << 32));
        
        if (!header) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: No command list allocated", port_number);
            return false;
        }

        HBACommandTable* table = reinterpret_cast<HBACommandTable*>(
            header[0].command_table_base_address |
            (static_cast<u64>(header[0].command_table_base_address_upper) << 32));
        
        if (!table) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: No command table allocated", port_number);
            return false;
        }

        FISRegisterH2D* fis = reinterpret_cast<FISRegisterH2D*>(table->command_fis);
        memset(fis, 0, sizeof(FISRegisterH2D));
        
        fis->fis_type = 0x27;
        fis->command_control = 1;
        fis->command = 0xEC;

        virt_addr buffer_virt = reinterpret_cast<virt_addr>(identify_data);
        phys_addr buffer_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(buffer_virt).value_or(0);
        
        if (!buffer_phys) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to get physical address for buffer", port_number);
            return false;
        }

        table->prdt_entries[0].data_base_address = buffer_phys & 0xFFFFFFFF;
        table->prdt_entries[0].data_base_address_upper = buffer_phys >> 32;
        table->prdt_entries[0].byte_count = 511;
        table->prdt_entries[0].interrupt_on_completion = 1;

        header[0].command_fis_length = sizeof(FISRegisterH2D) / sizeof(u32);
        header[0].write = 0;
        header[0].prdt_length = 1;
        header[0].clear_busy_on_ok = 1;

        AHCIPort& port = hba_->ports[port_number];
        port.interrupt_status = 0xFFFFFFFF;
        port.command_issue = 1 << 0;

        u32 timeout = 1000000;
        while (timeout--) {
            if ((port.command_issue & (1 << 0)) == 0) {
                break;
            }
            arch::CPU::pause();
        }

        if (port.command_issue & (1 << 0)) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: IDENTIFY command timeout", port_number);
            return false;
        }

        if (port.interrupt_status & (1 << 30)) {
            mm::VirtualMemoryManager::instance().kfree(identify_data);
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: IDENTIFY command failed", port_number);
            port.interrupt_status = 1 << 30;
            return false;
        }

        port.interrupt_status = 0xFFFFFFFF;

        u16* identify_words = reinterpret_cast<u16*>(identify_data);

        info.sector_size = 512;
        
        info.supports_48bit = (identify_words[83] & (1 << 10)) != 0;
        info.supports_ncq = (identify_words[76] & (1 << 8)) != 0;

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
            
            if (info.sector_count == 0xFFFFFFFF || info.sector_count == 0) {
                info.sector_count = 
                    (static_cast<u64>(identify_words[103]) << 0) |
                    (static_cast<u64>(identify_words[104]) << 16);
            }
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
            arch::CPU::pause();
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
            arch::CPU::pause();
        }
        
        return false;
    }

    bool AHCIController::send_command(u32 port_number, HBACommandHeader* header,
                                      HBACommandTable* table, void* buffer, usize buffer_size) {
        AHCIPort& port = hba_->ports[port_number];
        
        port.interrupt_status = 0xFFFFFFFF;
        
        u32 slot = 0;
        
        header->clear_busy_on_ok = 1;
        
        port.command_issue = 1 << slot;
        
        if (!wait_for_clear(port_number, offsetof(AHCIPort, command_issue), 
                           1 << slot, 1000000)) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Command timeout", port_number);
            return false;
        }
        
        if (port.interrupt_status & (1 << 30)) {
            port.interrupt_status = 1 << 30;
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Command failed with error", port_number);
            return false;
        }
        
        port.interrupt_status = 0xFFFFFFFF;
        
        return true;
    }

    bool AHCIController::read_sectors(u32 port_number, u64 lba, u32 count, void* buffer) {
        if (port_number >= ports_.size()) {
            return false;
        }

        ScopedLock lock(lock_);
        
        const PortInfo& info = ports_[port_number];
        
        if (!info.initialized) {
            return false;
        }

        if (lba + count > info.sector_count) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Read beyond end of disk (lba=%llu, count=%u, total=%llu)",
                      port_number, lba, count, info.sector_count);
            return false;
        }

        if (count == 0) {
            return true;
        }

        HBACommandHeader* header = reinterpret_cast<HBACommandHeader*>(
            reinterpret_cast<virt_addr>(hba_->ports[port_number].command_list_base) |
            (static_cast<u64>(hba_->ports[port_number].command_list_base_upper) << 32));
        
        if (!header) {
            return false;
        }

        HBACommandTable* table = reinterpret_cast<HBACommandTable*>(
            header[0].command_table_base_address |
            (static_cast<u64>(header[0].command_table_base_address_upper) << 32));
        
        if (!table) {
            return false;
        }

        memset(table, 0, sizeof(HBACommandTable) + 8 * sizeof(HBACommandTable::HBAPRDTEntry));

        FISRegisterH2D* fis = reinterpret_cast<FISRegisterH2D*>(table->command_fis);
        memset(fis, 0, sizeof(FISRegisterH2D));
        
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

        virt_addr buffer_virt = reinterpret_cast<virt_addr>(buffer);
        phys_addr buffer_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(buffer_virt).value_or(0);
        
        if (!buffer_phys) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to get physical address for buffer", port_number);
            return false;
        }

        usize total_bytes = count * info.sector_size;
        usize prdt_entries_needed = (total_bytes + 0x40000 - 1) / 0x40000;
        
        if (prdt_entries_needed > 8) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Transfer too large (%llu bytes, max %llu)",
                      port_number, total_bytes, 8 * 0x40000);
            return false;
        }

        usize remaining = total_bytes;
        phys_addr current_phys = buffer_phys;
        
        for (usize i = 0; i < prdt_entries_needed && remaining > 0; i++) {
            usize chunk_size = min(remaining, static_cast<usize>(0x40000));
            
            table->prdt_entries[i].data_base_address = current_phys & 0xFFFFFFFF;
            table->prdt_entries[i].data_base_address_upper = current_phys >> 32;
            table->prdt_entries[i].byte_count = chunk_size - 1;
            table->prdt_entries[i].interrupt_on_completion = (i == prdt_entries_needed - 1) ? 1 : 0;
            
            current_phys += chunk_size;
            remaining -= chunk_size;
        }

        header[0].command_fis_length = sizeof(FISRegisterH2D) / sizeof(u32);
        header[0].write = 0;
        header[0].prdt_length = prdt_entries_needed;
        header[0].clear_busy_on_ok = 1;

        return send_command(port_number, &header[0], table, buffer, total_bytes);
    }

    bool AHCIController::write_sectors(u32 port_number, u64 lba, u32 count, const void* buffer) {
        if (port_number >= ports_.size()) {
            return false;
        }

        ScopedLock lock(lock_);
        
        const PortInfo& info = ports_[port_number];
        
        if (!info.initialized) {
            return false;
        }

        if (lba + count > info.sector_count) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Write beyond end of disk (lba=%llu, count=%u, total=%llu)",
                      port_number, lba, count, info.sector_count);
            return false;
        }

        if (count == 0) {
            return true;
        }

        HBACommandHeader* header = reinterpret_cast<HBACommandHeader*>(
            reinterpret_cast<virt_addr>(hba_->ports[port_number].command_list_base) |
            (static_cast<u64>(hba_->ports[port_number].command_list_base_upper) << 32));
        
        if (!header) {
            return false;
        }

        HBACommandTable* table = reinterpret_cast<HBACommandTable*>(
            header[0].command_table_base_address |
            (static_cast<u64>(header[0].command_table_base_address_upper) << 32));
        
        if (!table) {
            return false;
        }

        memset(table, 0, sizeof(HBACommandTable) + 8 * sizeof(HBACommandTable::HBAPRDTEntry));

        FISRegisterH2D* fis = reinterpret_cast<FISRegisterH2D*>(table->command_fis);
        memset(fis, 0, sizeof(FISRegisterH2D));
        
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

        virt_addr buffer_virt = reinterpret_cast<virt_addr>(buffer);
        phys_addr buffer_phys = mm::VirtualMemoryManager::instance()
            .get_physical_address(buffer_virt).value_or(0);
        
        if (!buffer_phys) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Failed to get physical address for buffer", port_number);
            return false;
        }

        usize total_bytes = count * info.sector_size;
        usize prdt_entries_needed = (total_bytes + 0x40000 - 1) / 0x40000;
        
        if (prdt_entries_needed > 8) {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Port %u: Transfer too large (%llu bytes, max %llu)",
                      port_number, total_bytes, 8 * 0x40000);
            return false;
        }

        usize remaining = total_bytes;
        phys_addr current_phys = buffer_phys;
        
        for (usize i = 0; i < prdt_entries_needed && remaining > 0; i++) {
            usize chunk_size = min(remaining, static_cast<usize>(0x40000));
            
            table->prdt_entries[i].data_base_address = current_phys & 0xFFFFFFFF;
            table->prdt_entries[i].data_base_address_upper = current_phys >> 32;
            table->prdt_entries[i].byte_count = chunk_size - 1;
            table->prdt_entries[i].interrupt_on_completion = (i == prdt_entries_needed - 1) ? 1 : 0;
            
            current_phys += chunk_size;
            remaining -= chunk_size;
        }

        header[0].command_fis_length = sizeof(FISRegisterH2D) / sizeof(u32);
        header[0].write = 1;
        header[0].prdt_length = prdt_entries_needed;
        header[0].clear_busy_on_ok = 1;

        return send_command(port_number, &header[0], table, const_cast<void*>(buffer), total_bytes);
    }

    bool AHCIController::read(u32 port_number, u64 lba, u32 count, void* buffer) {
        return read_sectors(port_number, lba, count, buffer);
    }

    bool AHCIController::write(u32 port_number, u64 lba, u32 count, const void* buffer) {
        return write_sectors(port_number, lba, count, buffer);
    }

    const AHCIController::PortInfo* AHCIController::get_port_info(u32 index) const {
        ScopedLock lock(lock_);
        
        if (index >= ports_.size()) {
            return nullptr;
        }
        
        return &ports_[index];
    }

    void AHCIController::dump_info() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "AHCI", "AHCI Controller Information:");
        debug::log(debug::LogLevel::Info, "AHCI", "  Capabilities: 0x%08X", capabilities_);
        debug::log(debug::LogLevel::Info, "AHCI", "  Version: 0x%08X", version_);
        debug::log(debug::LogLevel::Info, "AHCI", "  Ports implemented: 0x%08X", ports_implemented_);
        debug::log(debug::LogLevel::Info, "AHCI", "  Active ports: %llu", ports_.size());
        
        for (usize i = 0; i < ports_.size(); i++) {
            const PortInfo& info = ports_[i];
            debug::log(debug::LogLevel::Info, "AHCI", "  Port %u:", info.number);
            debug::log(debug::LogLevel::Info, "AHCI", "    Model: %s", info.model);
            debug::log(debug::LogLevel::Info, "AHCI", "    Serial: %s", info.serial);
            debug::log(debug::LogLevel::Info, "AHCI", "    Firmware: %s", info.firmware);
            debug::log(debug::LogLevel::Info, "AHCI", "    Type: %u", info.type);
            debug::log(debug::LogLevel::Info, "AHCI", "    Size: %llu MB (%llu sectors)",
                      (info.sector_count * info.sector_size) / (1024 * 1024),
                      info.sector_count);
            debug::log(debug::LogLevel::Info, "AHCI", "    48-bit LBA: %s",
                      info.supports_48bit ? "yes" : "no");
            debug::log(debug::LogLevel::Info, "AHCI", "    NCQ: %s",
                      info.supports_ncq ? "yes" : "no");
        }
    }

    void AHCIManager::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "AHCI", "Initializing AHCI Manager");
        
        Vector<PCI::Device*> ahci_devices = PCI::find_devices_by_class(0x01, 0x06, 0x01);
        
        for (PCI::Device* device : ahci_devices) {
            debug::log(debug::LogLevel::Info, "AHCI",
                      "Found AHCI controller at %02X:%02X.%X",
                      device->get_bus(), device->get_slot(), device->get_function());
            
            add_controller(device);
        }
        
        debug::log(debug::LogLevel::Info, "AHCI", 
                  "AHCI Manager initialized with %llu controllers",
                  controllers_.size());
    }

    void AHCIManager::add_controller(PCI::Device* pci_device) {
        ScopedLock lock(lock_);
        
        AHCIController* controller = new AHCIController(pci_device);
        if (controller->init()) {
            controllers_.push_back(controller);
            controller->dump_info();
        } else {
            debug::log(debug::LogLevel::Error, "AHCI",
                      "Failed to initialize AHCI controller");
            delete controller;
        }
    }

    AHCIController* AHCIManager::get_controller(u32 index) {
        ScopedLock lock(lock_);
        
        if (index >= controllers_.size()) {
            return nullptr;
        }
        
        return controllers_[index];
    }

    bool AHCIManager::read(u32 controller_index, u32 port_index, 
                          u64 lba, u32 count, void* buffer) {
        ScopedLock lock(lock_);
        
        if (controller_index >= controllers_.size()) {
            return false;
        }
        
        AHCIController* controller = controllers_[controller_index];
        return controller->read(port_index, lba, count, buffer);
    }

    bool AHCIManager::write(u32 controller_index, u32 port_index,
                           u64 lba, u32 count, const void* buffer) {
        ScopedLock lock(lock_);
        
        if (controller_index >= controllers_.size()) {
            return false;
        }
        
        AHCIController* controller = controllers_[controller_index];
        return controller->write(port_index, lba, count, buffer);
    }
}
