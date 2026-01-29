#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/bitops.hpp>

namespace nk::mm {
    VirtualMemoryManager VirtualMemoryManager::instance_;

    VirtualMemoryManager& VirtualMemoryManager::instance() {
        return instance_;
    }

    VirtualMemoryManager::VirtualMemoryManager() 
        : kernel_space_(nullptr),
          current_space_(nullptr),
          kernel_heap_current_(0),
          kernel_heap_end_(0),
          heap_lock_() {}

    void VirtualMemoryManager::init() {
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        
        auto pml4_phys_opt = pmm.allocate_page();
        if (!pml4_phys_opt.has_value()) {
            panic("Failed to allocate PML4 for kernel address space");
        }
        
        phys_addr pml4_phys = pml4_phys_opt.value();
        PageTableEntry* pml4 = reinterpret_cast<PageTableEntry*>(
            phys_to_virt(pml4_phys));
        
        memset(pml4, 0, PAGE_SIZE);
        
        kernel_space_ = new AddressSpace();
        kernel_space_->pml4 = pml4;
        
        current_space_ = kernel_space_;
        
        kernel_heap_current_ = KERNEL_HEAP_BASE;
        kernel_heap_end_ = KERNEL_HEAP_BASE + KERNEL_HEAP_SIZE;
        
        map_kernel_regions();
        
        debug::log(debug::LogLevel::Info, "VMM", 
                  "Virtual memory manager initialized");
    }

    void VirtualMemoryManager::map_kernel_regions() {
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        
        virt_addr kernel_virtual = KERNEL_BASE;
        phys_addr kernel_physical = 0x100000;
        
        usize kernel_size = reinterpret_cast<usize>(&_kernel_end) - 
                           reinterpret_cast<usize>(&_kernel_start);
        kernel_size = align_up(kernel_size, PAGE_SIZE);
        
        usize pages = kernel_size / PAGE_SIZE;
        
        for (usize i = 0; i < pages; i++) {
            map_page(kernel_virtual + i * PAGE_SIZE,
                    kernel_physical + i * PAGE_SIZE,
                    PageFlags::Present | PageFlags::Writable | PageFlags::Global);
        }
        
        map_page(0xb8000, 0xb8000, 
                PageFlags::Present | PageFlags::Writable | PageFlags::CacheDisabled);
        
        debug::log(debug::LogLevel::Info, "VMM",
                  "Mapped kernel: 0x%016llX -> 0x%016llX (%llu pages)",
                  kernel_virtual, kernel_physical, pages);
    }

    VirtualMemoryManager::PageTableEntry* 
    VirtualMemoryManager::get_next_table(PageTableEntry* entry, usize level, bool allocate) {
        if (!entry->is_present()) {
            if (!allocate) {
                return nullptr;
            }
            
            PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
            auto page_opt = pmm.allocate_page();
            if (!page_opt.has_value()) {
                return nullptr;
            }
            
            phys_addr new_table_phys = page_opt.value();
            PageTableEntry* new_table = reinterpret_cast<PageTableEntry*>(
                phys_to_virt(new_table_phys));
            
            memset(new_table, 0, PAGE_SIZE);
            
            entry->raw = 0;
            entry->set_address(new_table_phys);
            entry->present = 1;
            entry->writable = 1;
            entry->user = (level == 0);
            
            current_space_->allocated_pages++;
        }
        
        phys_addr table_phys = entry->get_address();
        return reinterpret_cast<PageTableEntry*>(phys_to_virt(table_phys));
    }

    void VirtualMemoryManager::free_table(PageTableEntry* table, usize level) {
        if (level == 0) {
            return;
        }
        
        for (usize i = 0; i < PAGE_TABLE_ENTRIES; i++) {
            if (table[i].is_present()) {
                if (level > 1) {
                    phys_addr next_table_phys = table[i].get_address();
                    PageTableEntry* next_table = reinterpret_cast<PageTableEntry*>(
                        phys_to_virt(next_table_phys));
                    free_table(next_table, level - 1);
                }
                
                PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
                pmm.free_page(table[i].get_address());
                current_space_->allocated_pages--;
            }
        }
        
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        pmm.free_page(virt_to_phys(reinterpret_cast<virt_addr>(table)));
        current_space_->allocated_pages--;
    }

    Optional<phys_addr> VirtualMemoryManager::walk_page_table(virt_addr address, usize* level) {
        usize indices[PAGE_TABLE_LEVELS];
        indices[3] = (address >> 39) & 0x1FF;
        indices[2] = (address >> 30) & 0x1FF;
        indices[1] = (address >> 21) & 0x1FF;
        indices[0] = (address >> 12) & 0x1FF;
        
        PageTableEntry* current = current_space_->pml4;
        
        for (int i = 3; i >= 0; i--) {
            if (!current[indices[i]].is_present()) {
                if (level) *level = i;
                return Optional<phys_addr>();
            }
            
            if (i == 0) {
                if (level) *level = 0;
                return Optional<phys_addr>(current[indices[i]].get_address());
            }
            
            if (current[indices[i]].is_huge()) {
                if (level) *level = i;
                return Optional<phys_addr>(current[indices[i]].get_address());
            }
            
            phys_addr next_phys = current[indices[i]].get_address();
            current = reinterpret_cast<PageTableEntry*>(phys_to_virt(next_phys));
        }
        
        return Optional<phys_addr>();
    }

    bool VirtualMemoryManager::map_page_internal(virt_addr virt, phys_addr phys, PageFlags flags) {
        if (virt % PAGE_SIZE != 0 || phys % PAGE_SIZE != 0) {
            debug::log(debug::LogLevel::Error, "VMM",
                      "Unaligned address in map_page: virt=0x%016llX, phys=0x%016llX",
                      virt, phys);
            return false;
        }
        
        usize indices[PAGE_TABLE_LEVELS];
        indices[3] = (virt >> 39) & 0x1FF;
        indices[2] = (virt >> 30) & 0x1FF;
        indices[1] = (virt >> 21) & 0x1FF;
        indices[0] = (virt >> 12) & 0x1FF;
        
        PageTableEntry* current = current_space_->pml4;
        
        for (int i = 3; i > 0; i--) {
            PageTableEntry* next = get_next_table(&current[indices[i]], i, true);
            if (!next) {
                debug::log(debug::LogLevel::Error, "VMM",
                          "Failed to allocate page table level %d", i);
                return false;
            }
            current = next;
        }
        
        PageTableEntry& entry = current[indices[0]];
        if (entry.is_present()) {
            debug::log(debug::LogLevel::Warning, "VMM",
                      "Page already mapped: 0x%016llX -> 0x%016llX",
                      virt, entry.get_address());
            return false;
        }
        
        entry.raw = 0;
        entry.set_address(phys);
        entry.present = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::Present)) ? 1 : 0;
        entry.writable = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::Writable)) ? 1 : 0;
        entry.user = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::UserAccessible)) ? 1 : 0;
        entry.write_through = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::WriteThrough)) ? 1 : 0;
        entry.cache_disabled = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::CacheDisabled)) ? 1 : 0;
        entry.global = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::Global)) ? 1 : 0;
        entry.no_execute = (static_cast<u64>(flags) & static_cast<u64>(PageFlags::NoExecute)) ? 1 : 0;
        
        current_space_->mapped_pages++;
        
        invalidate_page(virt);
        
        return true;
    }

    bool VirtualMemoryManager::unmap_page_internal(virt_addr virt) {
        if (virt % PAGE_SIZE != 0) {
            debug::log(debug::LogLevel::Error, "VMM",
                      "Unaligned address in unmap_page: 0x%016llX", virt);
            return false;
        }
        
        usize indices[PAGE_TABLE_LEVELS];
        indices[3] = (virt >> 39) & 0x1FF;
        indices[2] = (virt >> 30) & 0x1FF;
        indices[1] = (virt >> 21) & 0x1FF;
        indices[0] = (virt >> 12) & 0x1FF;
        
        PageTableEntry* current = current_space_->pml4;
        PageTableEntry* tables[PAGE_TABLE_LEVELS];
        
        for (int i = 3; i >= 0; i--) {
            tables[i] = current;
            if (i == 0) break;
            
            if (!current[indices[i]].is_present()) {
                debug::log(debug::LogLevel::Warning, "VMM",
                          "Page not mapped: 0x%016llX", virt);
                return false;
            }
            
            phys_addr next_phys = current[indices[i]].get_address();
            current = reinterpret_cast<PageTableEntry*>(phys_to_virt(next_phys));
        }
        
        PageTableEntry& entry = tables[0][indices[0]];
        if (!entry.is_present()) {
            debug::log(debug::LogLevel::Warning, "VMM",
                      "Page not mapped: 0x%016llX", virt);
            return false;
        }
        
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        pmm.free_page(entry.get_address());
        
        entry.raw = 0;
        current_space_->mapped_pages--;
        
        invalidate_page(virt);
        
        for (int i = 1; i < PAGE_TABLE_LEVELS; i++) {
            PageTableEntry* table = tables[i];
            bool empty = true;
            
            for (usize j = 0; j < PAGE_TABLE_ENTRIES; j++) {
                if (table[j].is_present()) {
                    empty = false;
                    break;
                }
            }
            
            if (empty) {
                phys_addr table_phys = virt_to_phys(reinterpret_cast<virt_addr>(table));
                pmm.free_page(table_phys);
                current_space_->allocated_pages--;
                
                PageTableEntry& parent_entry = tables[i + 1][indices[i + 1]];
                parent_entry.raw = 0;
            }
        }
        
        return true;
    }

    void VirtualMemoryManager::invalidate_page(virt_addr address) {
        asm volatile("invlpg (%0)" : : "r"(address) : "memory");
    }

    bool VirtualMemoryManager::map_page(virt_addr virt, phys_addr phys, PageFlags flags) {
        ScopedLock lock(current_space_->lock);
        return map_page_internal(virt, phys, flags);
    }

    bool VirtualMemoryManager::map_pages(virt_addr virt, phys_addr phys, usize count, PageFlags flags) {
        ScopedLock lock(current_space_->lock);
        
        for (usize i = 0; i < count; i++) {
            if (!map_page_internal(virt + i * PAGE_SIZE, phys + i * PAGE_SIZE, flags)) {
                for (usize j = 0; j < i; j++) {
                    unmap_page_internal(virt + j * PAGE_SIZE);
                }
                return false;
            }
        }
        
        return true;
    }

    bool VirtualMemoryManager::unmap_page(virt_addr virt) {
        ScopedLock lock(current_space_->lock);
        return unmap_page_internal(virt);
    }

    bool VirtualMemoryManager::unmap_pages(virt_addr virt, usize count) {
        ScopedLock lock(current_space_->lock);
        
        bool success = true;
        for (usize i = 0; i < count; i++) {
            if (!unmap_page_internal(virt + i * PAGE_SIZE)) {
                success = false;
            }
        }
        
        return success;
    }

    Optional<phys_addr> VirtualMemoryManager::get_physical_address(virt_addr virt) {
        ScopedLock lock(current_space_->lock);
        
        usize level;
        auto phys_opt = walk_page_table(virt, &level);
        if (!phys_opt.has_value()) {
            return Optional<phys_addr>();
        }
        
        phys_addr phys = phys_opt.value();
        
        if (level > 0) {
            usize page_size = PAGE_SIZE;
            for (usize i = 0; i < level; i++) {
                page_size *= PAGE_TABLE_ENTRIES;
            }
            
            usize offset = virt & (page_size - 1);
            phys += offset;
        } else {
            phys += virt & (PAGE_SIZE - 1);
        }
        
        return Optional<phys_addr>(phys);
    }

    VirtualMemoryManager::AddressSpace* VirtualMemoryManager::create_address_space() {
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        
        auto pml4_phys_opt = pmm.allocate_page();
        if (!pml4_phys_opt.has_value()) {
            return nullptr;
        }
        
        phys_addr pml4_phys = pml4_phys_opt.value();
        PageTableEntry* pml4 = reinterpret_cast<PageTableEntry*>(
            phys_to_virt(pml4_phys));
        
        memset(pml4, 0, PAGE_SIZE);
        
        ScopedLock lock(kernel_space_->lock);
        
        for (usize i = 256; i < 512; i++) {
            pml4[i] = kernel_space_->pml4[i];
        }
        
        AddressSpace* space = new AddressSpace();
        space->pml4 = pml4;
        
        debug::log(debug::LogLevel::Debug, "VMM",
                  "Created new address space at 0x%016llX", pml4_phys);
        
        return space;
    }

    void VirtualMemoryManager::destroy_address_space(AddressSpace* space) {
        if (!space || space == kernel_space_) {
            return;
        }
        
        space->reference_count--;
        if (space->reference_count > 0) {
            return;
        }
        
        ScopedLock lock(space->lock);
        
        for (usize i = 0; i < 256; i++) {
            if (space->pml4[i].is_present()) {
                phys_addr pdpt_phys = space->pml4[i].get_address();
                PageTableEntry* pdpt = reinterpret_cast<PageTableEntry*>(
                    phys_to_virt(pdpt_phys));
                free_table(pdpt, 3);
            }
        }
        
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        pmm.free_page(virt_to_phys(reinterpret_cast<virt_addr>(space->pml4)));
        
        delete space;
        
        debug::log(debug::LogLevel::Debug, "VMM", "Destroyed address space");
    }

    void VirtualMemoryManager::switch_address_space(AddressSpace* space) {
        if (!space) {
            return;
        }
        
        ScopedLock lock(space->lock);
        
        phys_addr pml4_phys = virt_to_phys(reinterpret_cast<virt_addr>(space->pml4));
        asm volatile("mov %0, %%cr3" : : "r"(pml4_phys) : "memory");
        
        current_space_ = space;
        space->reference_count++;
    }

    void* VirtualMemoryManager::kmalloc(usize size) {
        if (size == 0) {
            return nullptr;
        }
        
        ScopedLock lock(heap_lock_);
        
        size = align_up(size, 16);
        
        if (kernel_heap_current_ + size > kernel_heap_end_) {
            debug::log(debug::LogLevel::Error, "VMM",
                      "Kernel heap exhausted: requested %llu bytes", size);
            return nullptr;
        }
        
        PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
        usize pages_needed = align_up(size, PAGE_SIZE) / PAGE_SIZE;
        
        for (usize i = 0; i < pages_needed; i++) {
            virt_addr virt = kernel_heap_current_ + i * PAGE_SIZE;
            auto phys_opt = pmm.allocate_page();
            if (!phys_opt.has_value()) {
                for (usize j = 0; j < i; j++) {
                    virt_addr free_virt = kernel_heap_current_ + j * PAGE_SIZE;
                    auto free_phys_opt = get_physical_address(free_virt);
                    if (free_phys_opt.has_value()) {
                        pmm.free_page(free_phys_opt.value());
                    }
                    unmap_page(free_virt);
                }
                return nullptr;
            }
            
            if (!map_page(virt, phys_opt.value(),
                         PageFlags::Present | PageFlags::Writable | PageFlags::Global)) {
                pmm.free_page(phys_opt.value());
                for (usize j = 0; j < i; j++) {
                    virt_addr free_virt = kernel_heap_current_ + j * PAGE_SIZE;
                    auto free_phys_opt = get_physical_address(free_virt);
                    if (free_phys_opt.has_value()) {
                        pmm.free_page(free_phys_opt.value());
                    }
                    unmap_page(free_virt);
                }
                return nullptr;
            }
        }
        
        void* ptr = reinterpret_cast<void*>(kernel_heap_current_);
        kernel_heap_current_ += pages_needed * PAGE_SIZE;
        
        return ptr;
    }

    void* VirtualMemoryManager::kmalloc_aligned(usize size, usize alignment) {
        if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
            return kmalloc(size);
        }
        
        alignment = max(alignment, static_cast<usize>(16));
        
        void* ptr = kmalloc(size + alignment - 1 + sizeof(void*));
        if (!ptr) {
            return nullptr;
        }
        
        uintptr_t raw = reinterpret_cast<uintptr_t>(ptr);
        uintptr_t aligned = (raw + sizeof(void*) + alignment - 1) & ~(alignment - 1);
        
        void** header = reinterpret_cast<void**>(aligned - sizeof(void*));
        *header = ptr;
        
        return reinterpret_cast<void*>(aligned);
    }

    void* VirtualMemoryManager::krealloc(void* ptr, usize new_size) {
        if (!ptr) {
            return kmalloc(new_size);
        }
        
        if (new_size == 0) {
            kfree(ptr);
            return nullptr;
        }
        
        usize old_size = 0;
        
        void** header = reinterpret_cast<void**>(
            reinterpret_cast<uintptr_t>(ptr) - sizeof(void*));
        
        if (reinterpret_cast<void*>(*header) != ptr) {
            void* new_ptr = kmalloc(new_size);
            if (!new_ptr) {
                return nullptr;
            }
            
            usize copy_size = min(old_size, new_size);
            memcpy(new_ptr, ptr, copy_size);
            
            kfree(ptr);
            return new_ptr;
        }
        
        void* new_ptr = kmalloc(new_size);
        if (!new_ptr) {
            return nullptr;
        }
        
        memcpy(new_ptr, ptr, min(old_size, new_size));
        kfree(ptr);
        
        return new_ptr;
    }

    void VirtualMemoryManager::kfree(void* ptr) {
        if (!ptr) {
            return;
        }
        
        ScopedLock lock(heap_lock_);
        
        void** header = reinterpret_cast<void**>(
            reinterpret_cast<uintptr_t>(ptr) - sizeof(void*));
        
        if (reinterpret_cast<void*>(*header) != ptr) {
            kfree(*header);
            return;
        }
        
        virt_addr virt = reinterpret_cast<virt_addr>(ptr);
        auto phys_opt = get_physical_address(virt);
        if (phys_opt.has_value()) {
            PhysicalMemoryManager& pmm = PhysicalMemoryManager::instance();
            pmm.free_page(phys_opt.value());
        }
        
        unmap_page(virt);
    }

    usize VirtualMemoryManager::get_allocated_pages() const {
        ScopedLock lock(current_space_->lock);
        return current_space_->allocated_pages;
    }

    usize VirtualMemoryManager::get_mapped_pages() const {
        ScopedLock lock(current_space_->lock);
        return current_space_->mapped_pages;
    }

    void VirtualMemoryManager::dump_page_tables(virt_addr start, virt_addr end) {
        ScopedLock lock(current_space_->lock);
        
        debug::log(debug::LogLevel::Info, "VMM", 
                  "Page table dump from 0x%016llX to 0x%016llX:", start, end);
        
        for (virt_addr addr = start; addr < end; addr += PAGE_SIZE) {
            usize level;
            auto phys_opt = walk_page_table(addr, &level);
            
            if (phys_opt.has_value()) {
                debug::log(debug::LogLevel::Info, "VMM",
                          "  0x%016llX -> 0x%016llX (level %llu)", 
                          addr, phys_opt.value(), level);
            }
        }
    }

    void VirtualMemoryManager::dump_memory_statistics() {
        debug::log(debug::LogLevel::Info, "VMM", "Virtual Memory Statistics:");
        debug::log(debug::LogLevel::Info, "VMM", "  Kernel Heap: 0x%016llX - 0x%016llX",
                  KERNEL_HEAP_BASE, kernel_heap_current_);
        debug::log(debug::LogLevel::Info, "VMM", "  Allocated pages: %llu", get_allocated_pages());
        debug::log(debug::LogLevel::Info, "VMM", "  Mapped pages: %llu", get_mapped_pages());
    }
}
