#ifndef NANOKOTON_VIRTUAL_HPP
#define NANOKOTON_VIRTUAL_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/mm/physical.hpp>
#include <nanokoton/lib/spinlock.hpp>

namespace nk::mm {
    enum class PageFlags : u64 {
        Present         = 1 << 0,
        Writable        = 1 << 1,
        UserAccessible  = 1 << 2,
        WriteThrough    = 1 << 3,
        CacheDisabled   = 1 << 4,
        Accessed        = 1 << 5,
        Dirty           = 1 << 6,
        HugePage        = 1 << 7,
        Global          = 1 << 8,
        NoExecute       = 1ULL << 63
    };

    inline PageFlags operator|(PageFlags a, PageFlags b) {
        return static_cast<PageFlags>(static_cast<u64>(a) | static_cast<u64>(b));
    }

    inline PageFlags operator&(PageFlags a, PageFlags b) {
        return static_cast<PageFlags>(static_cast<u64>(a) & static_cast<u64>(b));
    }

    class VirtualMemoryManager {
    private:
        struct PageTableEntry {
            union {
                struct {
                    u64 present : 1;
                    u64 writable : 1;
                    u64 user : 1;
                    u64 write_through : 1;
                    u64 cache_disabled : 1;
                    u64 accessed : 1;
                    u64 dirty : 1;
                    u64 huge : 1;
                    u64 global : 1;
                    u64 available1 : 3;
                    u64 address : 40;
                    u64 available2 : 11;
                    u64 no_execute : 1;
                };
                u64 raw;
            };

            PageTableEntry() : raw(0) {}
            
            bool is_present() const { return present; }
            bool is_writable() const { return writable; }
            bool is_user() const { return user; }
            bool is_huge() const { return huge; }
            bool is_global() const { return global; }
            bool no_execute() const { return no_execute; }
            
            phys_addr get_address() const { return address << 12; }
            void set_address(phys_addr addr) { address = addr >> 12; }
            
            void set_flags(PageFlags flags) {
                raw |= static_cast<u64>(flags);
            }
            
            void clear_flags(PageFlags flags) {
                raw &= ~static_cast<u64>(flags);
            }
            
            bool test_flags(PageFlags flags) const {
                return (raw & static_cast<u64>(flags)) == static_cast<u64>(flags);
            }
        };

        static constexpr usize PAGE_TABLE_LEVELS = 4;
        static constexpr usize PAGE_TABLE_ENTRIES = 512;
        static constexpr usize PAGE_SIZE = 4096;
        static constexpr usize HUGE_PAGE_SIZE = 2 * 1024 * 1024;
        
        static constexpr u64 KERNEL_BASE = 0xFFFFFFFF80000000;
        static constexpr u64 KERNEL_HEAP_BASE = 0xFFFF800000000000;
        static constexpr u64 KERNEL_HEAP_SIZE = 0x100000000;
        static constexpr u64 USER_BASE = 0x0000000000400000;
        static constexpr u64 USER_STACK_BASE = 0x00007FFFFFFFFFFF;
        static constexpr u64 USER_STACK_SIZE = 0x800000;

        struct AddressSpace {
            PageTableEntry* pml4;
            SpinLock lock;
            usize reference_count;
            u64 allocated_pages;
            u64 mapped_pages;
            
            AddressSpace() : pml4(nullptr), reference_count(1), allocated_pages(0), mapped_pages(0) {}
        };

        AddressSpace* kernel_space_;
        AddressSpace* current_space_;
        
        virt_addr kernel_heap_current_;
        virt_addr kernel_heap_end_;
        SpinLock heap_lock_;

        static VirtualMemoryManager instance_;

        VirtualMemoryManager();
        ~VirtualMemoryManager() = default;

        PageTableEntry* get_next_table(PageTableEntry* entry, usize level, bool allocate);
        void free_table(PageTableEntry* table, usize level);
        
        Optional<phys_addr> walk_page_table(virt_addr address, usize* level = nullptr);
        bool map_page_internal(virt_addr virt, phys_addr phys, PageFlags flags);
        bool unmap_page_internal(virt_addr virt);
        
        void invalidate_page(virt_addr address);

    public:
        static VirtualMemoryManager& instance();
        
        void init();
        
        bool map_page(virt_addr virt, phys_addr phys, PageFlags flags);
        bool map_pages(virt_addr virt, phys_addr phys, usize count, PageFlags flags);
        bool unmap_page(virt_addr virt);
        bool unmap_pages(virt_addr virt, usize count);
        
        Optional<phys_addr> get_physical_address(virt_addr virt);
        
        AddressSpace* create_address_space();
        void destroy_address_space(AddressSpace* space);
        void switch_address_space(AddressSpace* space);
        
        AddressSpace* get_kernel_space() { return kernel_space_; }
        AddressSpace* get_current_space() { return current_space_; }
        
        void* kmalloc(usize size);
        void* kmalloc_aligned(usize size, usize alignment);
        void* krealloc(void* ptr, usize new_size);
        void kfree(void* ptr);
        
        usize get_allocated_pages() const;
        usize get_mapped_pages() const;
        
        void dump_page_tables(virt_addr start, virt_addr end);
        void dump_memory_statistics();
        
        class ScopedAddressSpace {
        private:
            AddressSpace* old_space_;
        
        public:
            ScopedAddressSpace(AddressSpace* new_space) {
                old_space_ = VirtualMemoryManager::instance().current_space_;
                VirtualMemoryManager::instance().switch_address_space(new_space);
            }
            
            ~ScopedAddressSpace() {
                VirtualMemoryManager::instance().switch_address_space(old_space_);
            }
            
            ScopedAddressSpace(const ScopedAddressSpace&) = delete;
            ScopedAddressSpace& operator=(const ScopedAddressSpace&) = delete;
        };
    };
}
#endif
