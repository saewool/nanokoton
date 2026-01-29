#ifndef NANOKOTON_IDT_HPP
#define NANOKOTON_IDT_HPP

#include <nanokoton/types.hpp>

namespace nk::arch {
    struct PACKED IDTEntry {
        u16 offset_low;
        u16 selector;
        u8 ist;
        u8 type_attributes;
        u16 offset_mid;
        u32 offset_high;
        u32 reserved;
        
        IDTEntry() : offset_low(0), selector(0), ist(0), type_attributes(0),
                     offset_mid(0), offset_high(0), reserved(0) {}
        
        void set_offset(u64 offset);
        u64 get_offset() const;
        void set_present(bool present);
        void set_dpl(u8 dpl);
        void set_type(u8 type);
    };

    struct PACKED IDTPointer {
        u16 limit;
        u64 base;
        
        IDTPointer() : limit(0), base(0) {}
    };

    class InterruptDescriptorTable {
    private:
        static constexpr usize IDT_ENTRY_COUNT = 256;
        static IDTEntry entries_[IDT_ENTRY_COUNT];
        static IDTPointer pointer_;
        
        static void init_entries();
        static void load_idt();
        
    public:
        static void init();
        
        static void set_entry(u8 index, u64 handler, u16 selector, u8 ist, u8 type, u8 dpl);
        static void set_handler(u8 index, void (*handler)());
        
        static const IDTEntry& get_entry(u8 index);
        
        static void enable_interrupts();
        static void disable_interrupts();
        static bool interrupts_enabled();
        
        static void register_exception_handlers();
        static void register_interrupt_handlers();
        
        class ScopedInterruptDisable {
        private:
            bool was_enabled_;
        
        public:
            ScopedInterruptDisable();
            ~ScopedInterruptDisable();
            
            ScopedInterruptDisable(const ScopedInterruptDisable&) = delete;
            ScopedInterruptDisable& operator=(const ScopedInterruptDisable&) = delete;
        };
    };
}

extern "C" {
    void isr0();
    void isr1();
    void isr2();
    void isr3();
    void isr4();
    void isr5();
    void isr6();
    void isr7();
    void isr8();
    void isr9();
    void isr10();
    void isr11();
    void isr12();
    void isr13();
    void isr14();
    void isr15();
    void isr16();
    void isr17();
    void isr18();
    void isr19();
    void isr20();
    void isr21();
    void isr22();
    void isr23();
    void isr24();
    void isr25();
    void isr26();
    void isr27();
    void isr28();
    void isr29();
    void isr30();
    void isr31();
    
    void irq0();
    void irq1();
    void irq2();
    void irq3();
    void irq4();
    void irq5();
    void irq6();
    void irq7();
    void irq8();
    void irq9();
    void irq10();
    void irq11();
    void irq12();
    void irq13();
    void irq14();
    void irq15();
    
    void isr128();
    
    extern void* isr_stub_table[];
}

#endif
