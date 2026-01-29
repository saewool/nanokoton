#ifndef NANOKOTON_PHYSICAL_HPP
#define NANOKOTON_PHYSICAL_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/lib/bitmap.hpp>
#include <nanokoton/lib/spinlock.hpp>

namespace nk::mm {
    class PhysicalMemoryManager {
    private:
        struct MemoryRegion {
            phys_addr base;
            usize size;
            usize free_pages;
            usize total_pages;
            Bitmap* bitmap;
        };

        static constexpr usize PAGE_SIZE = 4096;
        static constexpr usize MAX_MEMORY_REGIONS = 32;
        static constexpr usize BITMAP_SIZE = 1024 * 1024;

        MemoryRegion regions_[MAX_MEMORY_REGIONS];
        usize region_count_;
        usize total_memory_;
        usize free_memory_;
        usize used_memory_;
        usize reserved_memory_;
        usize total_pages_;
        usize free_pages_;
        usize used_pages_;
        usize page_frames_allocated_;
        usize page_frames_freed_;

        Bitmap global_bitmap_;
        u8 bitmap_storage_[BITMAP_SIZE];
        SpinLock lock_;

        PhysicalMemoryManager();
        ~PhysicalMemoryManager() = default;

        bool init_region(phys_addr base, usize size);
        void mark_region(phys_addr base, usize size, bool used);
        Optional<usize> find_free_page_in_region(usize region_index);
        void merge_free_regions();

    public:
        static PhysicalMemoryManager& instance();

        void init(const BootInfo* boot_info);
        void init(phys_addr memory_map, usize entry_count);

        Optional<phys_addr> allocate_page();
        Optional<phys_addr> allocate_pages(usize count);
        Optional<phys_addr> allocate_aligned(usize count, usize alignment);

        void free_page(phys_addr page);
        void free_pages(phys_addr base, usize count);

        usize total_memory() const { return total_memory_; }
        usize free_memory() const { return free_memory_; }
        usize used_memory() const { return used_memory_; }
        usize reserved_memory() const { return reserved_memory_; }
        usize total_pages() const { return total_pages_; }
        usize free_pages() const { return free_pages_; }
        usize used_pages() const { return used_pages_; }

        usize page_frames_allocated() const { return page_frames_allocated_; }
        usize page_frames_freed() const { return page_frames_freed_; }

        phys_addr page_to_phys(usize page_index) const;
        usize phys_to_page(phys_addr address) const;

        bool is_page_free(phys_addr page) const;
        bool is_page_allocated(phys_addr page) const;
        bool is_page_reserved(phys_addr page) const;

        void dump_statistics() const;
        void dump_bitmap() const;

        void lock();
        void unlock();
        bool try_lock();

        class ScopedLock {
        private:
            PhysicalMemoryManager& pmm_;
        
        public:
            ScopedLock(PhysicalMemoryManager& pmm) : pmm_(pmm) {
                pmm_.lock();
            }
            
            ~ScopedLock() {
                pmm_.unlock();
            }
            
            ScopedLock(const ScopedLock&) = delete;
            ScopedLock& operator=(const ScopedLock&) = delete;
        };
    };
}

#endif
