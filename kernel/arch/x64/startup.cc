#include <nanokoton/types.hpp>
#include <nanokoton/core/kernel.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/arch/gdt.hpp>
#include <nanokoton/arch/idt.hpp>
#include <nanokoton/mm/physical.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/drivers/vga.hpp>
#include <nanokoton/drivers/serial.hpp>
#include <nanokoton/drivers/pic.hpp>
#include <nanokoton/drivers/pit.hpp>
#include <nanokoton/drivers/keyboard.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/stdio.hpp>

extern "C" u8 _kernel_start[];
extern "C" u8 _kernel_end[];

extern "C" NO_RETURN void _start(BootInfo* boot_info) {
    nk::Kernel::early_init(boot_info);
    nk::Kernel::init();
    nk::Kernel::run();
}

namespace nk {
    void Kernel::early_init(BootInfo* boot_info) {
        if (boot_info->magic_number != 0x4B4F544F4B4F4E4E) {
            panic("Invalid boot info magic number");
        }

        drivers::VGA::init();
        drivers::Serial::init(COM1);

        drivers::VGA::write_string("Nanokoton Kernel Early Initialization\n");
        drivers::Serial::write_string("Nanokoton Kernel Early Initialization\n");

        arch::CPU::init();
        arch::GDT::init();
        arch::IDT::init();

        drivers::PIC::remap(0x20, 0x28);
        drivers::PIT::init(1000);

        drivers::Keyboard::init();

        mm::PhysicalMemoryManager::init(boot_info);
        mm::VirtualMemoryManager::init();

        drivers::VGA::write_string("Early initialization complete\n");
    }

    void Kernel::init() {
        drivers::VGA::write_string("Initializing Nanokoton Kernel...\n");

        init_memory_management();
        init_interrupt_handling();
        init_device_drivers();
        init_filesystem_support();
        init_network_stack();
        init_process_management();
        init_system_calls();

        drivers::VGA::write_string("Kernel initialization complete\n");
        drivers::Serial::write_string("Kernel initialization complete\n");
    }

    void Kernel::run() {
        drivers::VGA::write_string("Nanokoton Kernel Running\n");

        arch::CPU::enable_interrupts();

        while (true) {
            asm volatile("hlt");
        }
    }

    void Kernel::init_memory_management() {
        mm::PhysicalMemoryManager& pmm = mm::PhysicalMemoryManager::instance();
        mm::VirtualMemoryManager& vmm = mm::VirtualMemoryManager::instance();

        vmm.map_kernel_regions();
        vmm.allocate_kernel_heap(0x1000000);

        drivers::VGA::write_string("Memory management initialized\n");
    }

    void Kernel::init_interrupt_handling() {
        for (u32 i = 0; i < 256; i++) {
            arch::IDT::set_handler(i, default_interrupt_handler);
        }

        arch::IDT::set_handler(0x20, drivers::PIT::interrupt_handler);
        arch::IDT::set_handler(0x21, drivers::Keyboard::interrupt_handler);
        arch::IDT::set_handler(0x0E, page_fault_handler);

        arch::IDT::load();

        drivers::VGA::write_string("Interrupt handling initialized\n");
    }

    void Kernel::init_device_drivers() {
        drivers::VGA::set_color(VGA_COLOR_LIGHT_GREEN, VGA_COLOR_BLACK);
        drivers::Serial::write_string("Device drivers initialized\n");

        drivers::PIT::start();
        drivers::Keyboard::enable();
    }

    void Kernel::init_filesystem_support() {
        fs::VirtualFileSystem::init();
        fs::ExFAT::init();

        drivers::VGA::write_string("Filesystem support initialized\n");
    }

    void Kernel::init_network_stack() {
        net::Ethernet::init();
        net::IP::init();
        net::TCP::init();
        net::UDP::init();

        drivers::VGA::write_string("Network stack initialized\n");
    }

    void Kernel::init_process_management() {
        task::Scheduler::init();
        task::ProcessManager::init();

        drivers::VGA::write_string("Process management initialized\n");
    }

    void Kernel::init_system_calls() {
        syscall::SystemCall::init();

        drivers::VGA::write_string("System calls initialized\n");
    }

    [[noreturn]] void Kernel::panic(const char* message) {
        arch::CPU::disable_interrupts();

        drivers::VGA::set_color(VGA_COLOR_WHITE, VGA_COLOR_RED);
        drivers::VGA::write_string("\n\nKERNEL PANIC: ");
        drivers::VGA::write_string(message);
        drivers::VGA::write_string("\n");

        drivers::Serial::write_string("\nKERNEL PANIC: ");
        drivers::Serial::write_string(message);
        drivers::Serial::write_string("\n");

        dump_debug_info();

        while (true) {
            arch::CPU::halt();
        }
    }

    void Kernel::dump_debug_info() {
        drivers::Serial::write_string("Debug Information:\n");
        drivers::Serial::write_string("Kernel Start: ");
        drivers::Serial::write_hex(reinterpret_cast<u64>(_kernel_start));
        drivers::Serial::write_string("\nKernel End: ");
        drivers::Serial::write_hex(reinterpret_cast<u64>(_kernel_end));
        drivers::Serial::write_string("\n");

        mm::PhysicalMemoryManager& pmm = mm::PhysicalMemoryManager::instance();
        drivers::Serial::write_string("Total Memory: ");
        drivers::Serial::write_decimal(pmm.total_memory());
        drivers::Serial::write_string(" bytes\n");
        drivers::Serial::write_string("Free Memory: ");
        drivers::Serial::write_decimal(pmm.free_memory());
        drivers::Serial::write_string(" bytes\n");
    }
}
