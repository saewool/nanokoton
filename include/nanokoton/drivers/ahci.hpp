#ifndef NANOKOTON_AHCI_HPP
#define NANOKOTON_AHCI_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/drivers/pci.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/spinlock.hpp>

namespace nk::drivers {
    struct PACKED AHCIPort {
        u64 command_list_base;
        u64 fis_base;
        u32 interrupt_status;
        u32 interrupt_enable;
        u32 command_status;
        u32 reserved0;
        u32 task_file_data;
        u32 signature;
        u32 sata_status;
        u32 sata_control;
        u32 sata_error;
        u32 sata_active;
        u32 command_issue;
        u32 sata_notification;
        u32 fis_switch_control;
        u32 reserved1[11];
        u32 vendor[4];
    };

    struct PACKED AHCIHostControl {
        u32 capabilities;
        u32 global_host_control;
        u32 interrupt_status;
        u32 ports_implemented;
        u32 version;
        u32 command_completion_coalescing_control;
        u32 command_completion_coalescing_ports;
        u32 enclosure_management_location;
        u32 enclosure_management_control;
        u32 host_capabilities_extended;
        u32 bios_handoff_control_status;
        u8 reserved0[0x74];
        u8 vendor[0x60];
        AHCIPort ports[32];
    };

    struct PACKED HBACommandHeader {
        u8 command_fis_length : 5;
        u8 atapi : 1;
        u8 write : 1;
        u8 prefetchable : 1;
        
        u8 reset : 1;
        u8 bist : 1;
        u8 clear_busy_on_ok : 1;
        u8 reserved0 : 1;
        u8 port_multiplier : 4;
        
        u16 prdt_length;
        u32 prdb_byte_count;
        u32 command_table_base_address;
        u32 command_table_base_address_upper;
        u32 reserved1[4];
    };

    struct PACKED HBACommandTable {
        u8 command_fis[64];
        u8 atapi_command[16];
        u8 reserved[48];
        struct PACKED HBAPRDTEntry {
            u32 data_base_address;
            u32 data_base_address_upper;
            u32 reserved0;
            u32 byte_count : 22;
            u32 reserved1 : 9;
            u32 interrupt_on_completion : 1;
        } prdt_entries[];
    };

    struct PACKED FISRegisterH2D {
        u8 fis_type;
        u8 port_multiplier : 4;
        u8 reserved0 : 3;
        u8 command_control : 1;
        u8 command;
        u8 feature_low;
        
        u8 lba0;
        u8 lba1;
        u8 lba2;
        u8 device;
        
        u8 lba3;
        u8 lba4;
        u8 lba5;
        u8 feature_high;
        
        u8 count_low;
        u8 count_high;
        u8 icc;
        u8 control;
        
        u8 reserved1[4];
    };

    struct PACKED FISRegisterD2H {
        u8 fis_type;
        u8 port_multiplier : 4;
        u8 reserved0 : 2;
        u8 interrupt : 1;
        u8 reserved1 : 1;
        u8 status;
        u8 error;
        
        u8 lba0;
        u8 lba1;
        u8 lba2;
        u8 device;
        
        u8 lba3;
        u8 lba4;
        u8 lba5;
        u8 reserved2;
        
        u8 count_low;
        u8 count_high;
        u8 reserved3[2];
        
        u8 reserved4[4];
    };

    struct PACKED FISData {
        u8 fis_type;
        u8 port_multiplier : 4;
        u8 reserved0 : 4;
        u8 reserved1[2];
        
        u32 data[0];
    };

    struct PACKED FISPIOSetup {
        u8 fis_type;
        u8 port_multiplier : 4;
        u8 reserved0 : 1;
        u8 data_direction : 1;
        u8 interrupt : 1;
        u8 reserved1 : 1;
        
        u8 status;
        u8 error;
        
        u8 lba0;
        u8 lba1;
        u8 lba2;
        u8 device;
        
        u8 lba3;
        u8 lba4;
        u8 lba5;
        u8 reserved2;
        
        u8 count_low;
        u8 count_high;
        u8 reserved3;
        u8 e_status;
        
        u16 transfer_count;
        u8 reserved4[2];
    };

    struct PACKED FISDMASetup {
        u8 fis_type;
        u8 port_multiplier : 4;
        u8 reserved0 : 1;
        u8 data_direction : 1;
        u8 interrupt : 1;
        u8 auto_activate : 1;
        u8 reserved1[2];
        
        u64 dma_buffer_id;
        
        u32 reserved2;
        u32 dma_buffer_offset;
        u32 transfer_count;
        u32 reserved3;
    };

    class AHCIController {
    private:
        PCI::Device* pci_device_;
        AHCIHostControl* hba_;
        u32 capabilities_;
        u32 ports_implemented_;
        u32 version_;
        
        struct PortInfo {
            u32 number;
            u32 type;
            u64 sector_count;
            u32 sector_size;
            bool supports_48bit;
            bool supports_ncq;
            char model[41];
            char serial[21];
            char firmware[9];
            bool initialized;
        };
        
        Vector<PortInfo> ports_;
        SpinLock lock_;
        
        bool find_device();
        bool init_hba();
        bool probe_port(u32 port_number);
        bool reset_port(u32 port_number);
        bool start_port(u32 port_number);
        bool stop_port(u32 port_number);
        
        bool identify_device(u32 port_number, PortInfo& info);
        bool read_sectors(u32 port_number, u64 lba, u32 count, void* buffer);
        bool write_sectors(u32 port_number, u64 lba, u32 count, const void* buffer);
        
        bool wait_for_clear(u32 port_number, u32 offset, u32 mask, u32 timeout);
        bool wait_for_set(u32 port_number, u32 offset, u32 mask, u32 timeout);
        
        bool send_command(u32 port_number, HBACommandHeader* header, 
                         HBACommandTable* table, void* buffer, usize buffer_size);
        
    public:
        AHCIController(PCI::Device* pci_device);
        ~AHCIController();
        
        bool init();
        bool read(u32 port_number, u64 lba, u32 count, void* buffer);
        bool write(u32 port_number, u64 lba, u32 count, const void* buffer);
        
        usize get_port_count() const { return ports_.size(); }
        const PortInfo* get_port_info(u32 index) const;
        
        void dump_info() const;
    };
    
    class AHCIManager {
    private:
        static Vector<AHCIController*> controllers_;
        static SpinLock lock_;
        
    public:
        static void init();
        static void add_controller(PCI::Device* pci_device);
        static AHCIController* get_controller(u32 index);
        static usize get_controller_count() { return controllers_.size(); }
        
        static bool read(u32 controller_index, u32 port_index, 
                        u64 lba, u32 count, void* buffer);
        static bool write(u32 controller_index, u32 port_index,
                         u64 lba, u32 count, const void* buffer);
    };
}
#endif
