#include <nanokoton/mm/physical.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/bitops.hpp>
#include <nanokoton/arch/cpu.hpp>

namespace nk::mm {
    PhysicalMemoryManager& PhysicalMemoryManager::instance() {
        static PhysicalMemoryManager instance;
        return instance;
    }

    PhysicalMemoryManager::PhysicalMemoryManager() 
        : region_count_(0),
          total_memory_(0),
          free_memory_(0),
          used_memory_(0),
          reserved_memory_(0),
          total_pages_(0),
          free_pages_(0),
          used_pages_(0),
          page_frames_allocated_(0),
          page_frames_freed_(0),
          global_bitmap_(nullptr, 0),
          lock_() {
        
        memset(regions_, 0, sizeof(regions_));
        memset(bitmap_storage_, 0, sizeof(bitmap_storage_));
    }

    void PhysicalMemoryManager::init(const BootInfo* boot_info) {
        ScopedLock lock(*this);

        if (!boot_info || boot_info->magic_number != 0x4B4F544F4B4F4E4E) {
            panic("Invalid boot info in PhysicalMemoryManager::init");
        }

        MemoryMapEntry* entries = reinterpret_cast<MemoryMapEntry*>(boot_info->memory_map_address);
        usize entry_count = boot_info->memory_map_entry_count;

        init(entries, entry_count);
    }

    void PhysicalMemoryManager::init(phys_addr memory_map, usize entry_count) {
        ScopedLock lock(*this);

        MemoryMapEntry* entries = reinterpret_cast<MemoryMapEntry*>(memory_map);
        
        debug::log(debug::LogLevel::Info, "PMM", 
                  "Initializing physical memory manager with %llu entries", entry_count);

        global_bitmap_ = Bitmap(bitmap_storage_, BITMAP_SIZE * 8);

        for (usize i = 0; i < entry_count; i++) {
            const MemoryMapEntry& entry = entries[i];
            
            if (entry.region_type == MemoryType::Free) {
                if (init_region(entry.base_address, entry.region_length)) {
                    debug::log(debug::LogLevel::Debug, "PMM",
                              "Added free region: 0x%016llX - 0x%016llX (%llu MB)",
                              entry.base_address,
                              entry.base_address + entry.region_length,
                              entry.region_length / (1024 * 1024));
                }
            } else {
                mark_region(entry.base_address, entry.region_length, true);
                
                if (entry.region_type == MemoryType::Reserved ||
                    entry.region_type == MemoryType::BadMemory) {
                    reserved_memory_ += entry.region_length;
                }
            }
        }

        total_memory_ = 0;
        free_memory_ = 0;
        used_memory_ = 0;
        total_pages_ = 0;
        free_pages_ = 0;
        used_pages_ = 0;

        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            total_memory_ += region.size;
            free_memory_ += region.free_pages * PAGE_SIZE;
            total_pages_ += region.total_pages;
            free_pages_ += region.free_pages;
        }

        used_pages_ = total_pages_ - free_pages_;
        used_memory_ = total_memory_ - free_memory_;

        merge_free_regions();

        debug::log(debug::LogLevel::Info, "PMM", 
                  "Memory statistics: Total=%llu MB, Free=%llu MB, Used=%llu MB, Reserved=%llu MB",
                  total_memory_ / (1024 * 1024),
                  free_memory_ / (1024 * 1024),
                  used_memory_ / (1024 * 1024),
                  reserved_memory_ / (1024 * 1024));
        
        debug::log(debug::LogLevel::Info, "PMM",
                  "Page statistics: Total=%llu, Free=%llu, Used=%llu",
                  total_pages_, free_pages_, used_pages_);
    }

    bool PhysicalMemoryManager::init_region(phys_addr base, usize size) {
        if (region_count_ >= MAX_MEMORY_REGIONS) {
            debug::log(debug::LogLevel::Error, "PMM", 
                      "Maximum memory regions exceeded");
            return false;
        }

        if (size < PAGE_SIZE) {
            return false;
        }

        base = align_up(base, PAGE_SIZE);
        size = align_down(size, PAGE_SIZE);

        if (size == 0) {
            return false;
        }

        MemoryRegion& region = regions_[region_count_];
        region.base = base;
        region.size = size;
        region.total_pages = size / PAGE_SIZE;
        
        usize bitmap_size = region.total_pages / 8;
        if (region.total_pages % 8 != 0) {
            bitmap_size++;
        }

        static usize current_bitmap_offset = 0;
        if (current_bitmap_offset + bitmap_size > sizeof(bitmap_storage_)) {
            debug::log(debug::LogLevel::Error, "PMM", 
                      "Bitmap storage exhausted");
            return false;
        }

        region.bitmap = new (bitmap_storage_ + current_bitmap_offset) Bitmap(
            bitmap_storage_ + current_bitmap_offset, region.total_pages);
        current_bitmap_offset += bitmap_size;

        region.bitmap->set_all(false);
        region.free_pages = region.total_pages;

        region_count_++;
        return true;
    }

    void PhysicalMemoryManager::mark_region(phys_addr base, usize size, bool used) {
        base = align_up(base, PAGE_SIZE);
        size = align_down(size, PAGE_SIZE);

        if (size == 0) {
            return;
        }

        for (usize i = 0; i < region_count_; i++) {
            MemoryRegion& region = regions_[i];
            
            if (base >= region.base && base < region.base + region.size) {
                usize start_page = (base - region.base) / PAGE_SIZE;
                usize end_page = start_page + (size / PAGE_SIZE);
                
                if (end_page > region.total_pages) {
                    end_page = region.total_pages;
                }

                for (usize page = start_page; page < end_page; page++) {
                    bool current_state = region.bitmap->test(page);
                    region.bitmap->set(page, used);
                    
                    if (current_state != used) {
                        if (used) {
                            region.free_pages--;
                            used_pages_++;
                            free_pages_--;
                            used_memory_ += PAGE_SIZE;
                            free_memory_ -= PAGE_SIZE;
                        } else {
                            region.free_pages++;
                            used_pages_--;
                            free_pages_++;
                            used_memory_ -= PAGE_SIZE;
                            free_memory_ += PAGE_SIZE;
                        }
                    }
                }
            }
        }
    }

    Optional<usize> PhysicalMemoryManager::find_free_page_in_region(usize region_index) {
        if (region_index >= region_count_) {
            return Optional<usize>();
        }

        MemoryRegion& region = regions_[region_index];
        
        for (usize page = 0; page < region.total_pages; page++) {
            if (!region.bitmap->test(page)) {
                return Optional<usize>(page);
            }
        }

        return Optional<usize>();
    }

    void PhysicalMemoryManager::merge_free_regions() {
        for (usize i = 0; i < region_count_; i++) {
            for (usize j = i + 1; j < region_count_; j++) {
                MemoryRegion& region1 = regions_[i];
                MemoryRegion& region2 = regions_[j];

                if (region1.base + region1.size == region2.base) {
                    region1.size += region2.size;
                    region1.total_pages += region2.total_pages;
                    region1.free_pages += region2.free_pages;

                    for (usize k = j; k < region_count_ - 1; k++) {
                        regions_[k] = regions_[k + 1];
                    }
                    
                    region_count_--;
                    j--;
                }
            }
        }
    }

    Optional<phys_addr> PhysicalMemoryManager::allocate_page() {
        ScopedLock lock(*this);

        for (usize i = 0; i < region_count_; i++) {
            auto page_opt = find_free_page_in_region(i);
            if (page_opt.has_value()) {
                usize page = page_opt.value();
                MemoryRegion& region = regions_[i];
                
                region.bitmap->set(page, true);
                region.free_pages--;
                free_pages_--;
                used_pages_++;
                free_memory_ -= PAGE_SIZE;
                used_memory_ += PAGE_SIZE;
                page_frames_allocated_++;

                phys_addr address = region.base + page * PAGE_SIZE;
                
                debug::log(debug::LogLevel::Trace, "PMM",
                          "Allocated page at 0x%016llX", address);
                
                return Optional<phys_addr>(address);
            }
        }

        debug::log(debug::LogLevel::Error, "PMM", 
                  "Out of memory: failed to allocate page");
        
        return Optional<phys_addr>();
    }

    Optional<phys_addr> PhysicalMemoryManager::allocate_pages(usize count) {
        if (count == 0) {
            return Optional<phys_addr>();
        }

        ScopedLock lock(*this);

        for (usize i = 0; i < region_count_; i++) {
            MemoryRegion& region = regions_[i];
            
            if (region.free_pages < count) {
                continue;
            }

            usize consecutive_count = 0;
            usize start_page = 0;

            for (usize page = 0; page < region.total_pages; page++) {
                if (!region.bitmap->test(page)) {
                    if (consecutive_count == 0) {
                        start_page = page;
                    }
                    consecutive_count++;
                    
                    if (consecutive_count == count) {
                        for (usize p = start_page; p < start_page + count; p++) {
                            region.bitmap->set(p, true);
                        }
                        
                        region.free_pages -= count;
                        free_pages_ -= count;
                        used_pages_ += count;
                        free_memory_ -= count * PAGE_SIZE;
                        used_memory_ += count * PAGE_SIZE;
                        page_frames_allocated_ += count;

                        phys_addr address = region.base + start_page * PAGE_SIZE;
                        
                        debug::log(debug::LogLevel::Trace, "PMM",
                                  "Allocated %llu pages starting at 0x%016llX", 
                                  count, address);
                        
                        return Optional<phys_addr>(address);
                    }
                } else {
                    consecutive_count = 0;
                }
            }
        }

        debug::log(debug::LogLevel::Error, "PMM", 
                  "Out of memory: failed to allocate %llu pages", count);
        
        return Optional<phys_addr>();
    }

    Optional<phys_addr> PhysicalMemoryManager::allocate_aligned(usize count, usize alignment) {
        if (count == 0) {
            return Optional<phys_addr>();
        }

        alignment = max(alignment, PAGE_SIZE);
        if (alignment % PAGE_SIZE != 0) {
            alignment = align_up(alignment, PAGE_SIZE);
        }

        ScopedLock lock(*this);

        for (usize i = 0; i < region_count_; i++) {
            MemoryRegion& region = regions_[i];
            
            if (region.free_pages < count) {
                continue;
            }

            usize consecutive_count = 0;
            usize start_page = 0;

            for (usize page = 0; page < region.total_pages; page++) {
                phys_addr candidate_addr = region.base + page * PAGE_SIZE;
                
                if (candidate_addr % alignment == 0) {
                    if (!region.bitmap->test(page)) {
                        if (consecutive_count == 0) {
                            start_page = page;
                        }
                        consecutive_count++;
                        
                        if (consecutive_count == count) {
                            bool all_free = true;
                            for (usize p = start_page; p < start_page + count; p++) {
                                if (region.bitmap->test(p)) {
                                    all_free = false;
                                    break;
                                }
                            }
                            
                            if (all_free) {
                                for (usize p = start_page; p < start_page + count; p++) {
                                    region.bitmap->set(p, true);
                                }
                                
                                region.free_pages -= count;
                                free_pages_ -= count;
                                used_pages_ += count;
                                free_memory_ -= count * PAGE_SIZE;
                                used_memory_ += count * PAGE_SIZE;
                                page_frames_allocated_ += count;

                                phys_addr address = region.base + start_page * PAGE_SIZE;
                                
                                debug::log(debug::LogLevel::Trace, "PMM",
                                          "Allocated %llu aligned pages at 0x%016llX (alignment: 0x%llX)", 
                                          count, address, alignment);
                                
                                return Optional<phys_addr>(address);
                            }
                        }
                    } else {
                        consecutive_count = 0;
                    }
                } else {
                    consecutive_count = 0;
                }
            }
        }

        debug::log(debug::LogLevel::Error, "PMM", 
                  "Out of memory: failed to allocate %llu aligned pages (alignment: 0x%llX)", 
                  count, alignment);
        
        return Optional<phys_addr>();
    }

    void PhysicalMemoryManager::free_page(phys_addr page) {
        if (page % PAGE_SIZE != 0) {
            debug::log(debug::LogLevel::Warning, "PMM",
                      "Attempt to free unaligned page: 0x%016llX", page);
            return;
        }

        ScopedLock lock(*this);

        for (usize i = 0; i < region_count_; i++) {
            MemoryRegion& region = regions_[i];
            
            if (page >= region.base && page < region.base + region.size) {
                usize page_index = (page - region.base) / PAGE_SIZE;
                
                if (page_index >= region.total_pages) {
                    debug::log(debug::LogLevel::Error, "PMM",
                              "Page index out of range: 0x%016llX", page);
                    return;
                }

                if (!region.bitmap->test(page_index)) {
                    debug::log(debug::LogLevel::Warning, "PMM",
                              "Double free detected: 0x%016llX", page);
                    return;
                }

                region.bitmap->set(page_index, false);
                region.free_pages++;
                free_pages_++;
                used_pages_--;
                free_memory_ += PAGE_SIZE;
                used_memory_ -= PAGE_SIZE;
                page_frames_freed_++;

                debug::log(debug::LogLevel::Trace, "PMM",
                          "Freed page at 0x%016llX", page);
                
                return;
            }
        }

        debug::log(debug::LogLevel::Error, "PMM",
                  "Attempt to free unknown page: 0x%016llX", page);
    }

    void PhysicalMemoryManager::free_pages(phys_addr base, usize count) {
        if (base % PAGE_SIZE != 0) {
            debug::log(debug::LogLevel::Warning, "PMM",
                      "Attempt to free unaligned pages: 0x%016llX", base);
            return;
        }

        if (count == 0) {
            return;
        }

        ScopedLock lock(*this);

        for (usize i = 0; i < region_count_; i++) {
            MemoryRegion& region = regions_[i];
            
            if (base >= region.base && base < region.base + region.size) {
                usize start_page = (base - region.base) / PAGE_SIZE;
                
                if (start_page + count > region.total_pages) {
                    debug::log(debug::LogLevel::Error, "PMM",
                              "Page range out of region: 0x%016llX + %llu pages", 
                              base, count);
                    return;
                }

                for (usize p = 0; p < count; p++) {
                    usize page_index = start_page + p;
                    
                    if (!region.bitmap->test(page_index)) {
                        debug::log(debug::LogLevel::Warning, "PMM",
                                  "Double free detected in page range: 0x%016llX", 
                                  base + p * PAGE_SIZE);
                    } else {
                        region.bitmap->set(page_index, false);
                    }
                }

                region.free_pages += count;
                free_pages_ += count;
                used_pages_ -= count;
                free_memory_ += count * PAGE_SIZE;
                used_memory_ -= count * PAGE_SIZE;
                page_frames_freed_ += count;

                debug::log(debug::LogLevel::Trace, "PMM",
                          "Freed %llu pages starting at 0x%016llX", count, base);
                
                return;
            }
        }

        debug::log(debug::LogLevel::Error, "PMM",
                  "Attempt to free unknown pages: 0x%016llX + %llu pages", base, count);
    }

    phys_addr PhysicalMemoryManager::page_to_phys(usize page_index) const {
        usize current_page = 0;
        
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            
            if (page_index < current_page + region.total_pages) {
                usize region_page = page_index - current_page;
                return region.base + region_page * PAGE_SIZE;
            }
            
            current_page += region.total_pages;
        }
        
        return 0;
    }

    usize PhysicalMemoryManager::phys_to_page(phys_addr address) const {
        usize current_page = 0;
        
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            
            if (address >= region.base && address < region.base + region.size) {
                usize region_page = (address - region.base) / PAGE_SIZE;
                return current_page + region_page;
            }
            
            current_page += region.total_pages;
        }
        
        return static_cast<usize>(-1);
    }

    bool PhysicalMemoryManager::is_page_free(phys_addr page) const {
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            
            if (page >= region.base && page < region.base + region.size) {
                usize page_index = (page - region.base) / PAGE_SIZE;
                return !region.bitmap->test(page_index);
            }
        }
        
        return false;
    }

    bool PhysicalMemoryManager::is_page_allocated(phys_addr page) const {
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            
            if (page >= region.base && page < region.base + region.size) {
                usize page_index = (page - region.base) / PAGE_SIZE;
                return region.bitmap->test(page_index);
            }
        }
        
        return false;
    }

    bool PhysicalMemoryManager::is_page_reserved(phys_addr page) const {
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            
            if (page >= region.base && page < region.base + region.size) {
                usize page_index = (page - region.base) / PAGE_SIZE;
                return region.bitmap->test(page_index);
            }
        }
        
        return true;
    }

    void PhysicalMemoryManager::dump_statistics() const {
        debug::log(debug::LogLevel::Info, "PMM", "Physical Memory Statistics:");
        debug::log(debug::LogLevel::Info, "PMM", "  Total Memory: %llu MB", total_memory_ / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "PMM", "  Free Memory:  %llu MB", free_memory_ / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "PMM", "  Used Memory:  %llu MB", used_memory_ / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "PMM", "  Reserved:     %llu MB", reserved_memory_ / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "PMM", "  Total Pages:  %llu", total_pages_);
        debug::log(debug::LogLevel::Info, "PMM", "  Free Pages:   %llu", free_pages_);
        debug::log(debug::LogLevel::Info, "PMM", "  Used Pages:   %llu", used_pages_);
        debug::log(debug::LogLevel::Info, "PMM", "  Allocations:  %llu", page_frames_allocated_);
        debug::log(debug::LogLevel::Info, "PMM", "  Frees:        %llu", page_frames_freed_);
        debug::log(debug::LogLevel::Info, "PMM", "  Regions:      %llu", region_count_);
        
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            debug::log(debug::LogLevel::Info, "PMM", 
                      "  Region %llu: 0x%016llX - 0x%016llX (%llu MB, %llu/%llu pages free)",
                      i, region.base, region.base + region.size,
                      region.size / (1024 * 1024),
                      region.free_pages, region.total_pages);
        }
    }

    void PhysicalMemoryManager::dump_bitmap() const {
        debug::log(debug::LogLevel::Debug, "PMM", "Physical Memory Bitmap:");
        
        for (usize i = 0; i < region_count_; i++) {
            const MemoryRegion& region = regions_[i];
            debug::log(debug::LogLevel::Debug, "PMM",
                      "Region %llu bitmap (first 256 bits):", i);
            
            char line[256];
            usize offset = 0;
            
            for (usize bit = 0; bit < 256 && bit < region.total_pages; bit++) {
                if (bit % 64 == 0 && bit != 0) {
                    line[offset] = '\0';
                    debug::log(debug::LogLevel::Debug, "PMM", "  %s", line);
                    offset = 0;
                }
                
                line[offset++] = region.bitmap->test(bit) ? '1' : '0';
            }
            
            if (offset > 0) {
                line[offset] = '\0';
                debug::log(debug::LogLevel::Debug, "PMM", "  %s", line);
            }
        }
    }

    void PhysicalMemoryManager::lock() {
        lock_.acquire();
    }

    void PhysicalMemoryManager::unlock() {
        lock_.release();
    }

    bool PhysicalMemoryManager::try_lock() {
        return lock_.try_acquire();
    }
}
