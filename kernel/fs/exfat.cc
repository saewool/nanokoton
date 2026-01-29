#include <nanokoton/fs/exfat.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/lib/bitops.hpp>
#include <nanokoton/lib/algorithm.hpp>
#include <nanokoton/arch/cpu.hpp>

namespace nk::fs {
    exFATVolume::exFATVolume(drivers::AHCIController* controller, u32 port_index, u64 partition_start)
        : controller_(controller),
          port_index_(port_index),
          partition_start_(partition_start),
          bytes_per_sector_(0),
          sectors_per_cluster_(0),
          bytes_per_cluster_(0),
          total_clusters_(0),
          fat_start_sector_(0),
          fat_size_sectors_(0),
          cluster_heap_start_sector_(0),
          root_dir_cluster_(0),
          cluster_bitmap_(nullptr, 0),
          cache_hits_(0),
          cache_misses_(0) {
        memset(&bs_, 0, sizeof(bs_));
    }

    exFATVolume::~exFATVolume() {
        flush_cache();
        
        if (cluster_bitmap_.data()) {
            mm::VirtualMemoryManager::instance().kfree(cluster_bitmap_.data());
        }
    }

    bool exFATVolume::init() {
        if (!read_sector(partition_start_, &bs_)) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Failed to read boot sector from partition start 0x%016llX",
                      partition_start_);
            return false;
        }

        if (bs_.boot_signature != 0xAA55) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Invalid boot signature: 0x%04X", bs_.boot_signature);
            return false;
        }

        if (memcmp(bs_.file_system_name, "EXFAT   ", 8) != 0) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Not an exFAT filesystem");
            return false;
        }

        bytes_per_sector_ = 1 << bs_.bytes_per_sector_shift;
        sectors_per_cluster_ = 1 << bs_.sectors_per_cluster_shift;
        bytes_per_cluster_ = bytes_per_sector_ * sectors_per_cluster_;
        total_clusters_ = bs_.cluster_count;
        fat_start_sector_ = bs_.fat_offset;
        fat_size_sectors_ = bs_.fat_length;
        cluster_heap_start_sector_ = bs_.cluster_heap_offset;
        root_dir_cluster_ = bs_.first_cluster_of_root_directory;

        if (bytes_per_sector_ < 512 || bytes_per_sector_ > 4096) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Invalid sector size: %u bytes", bytes_per_sector_);
            return false;
        }

        if (sectors_per_cluster_ == 0 || (sectors_per_cluster_ & (sectors_per_cluster_ - 1)) != 0) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Invalid sectors per cluster: %u", sectors_per_cluster_);
            return false;
        }

        usize bitmap_size = (total_clusters_ + 7) / 8;
        u8* bitmap_data = reinterpret_cast<u8*>(
            mm::VirtualMemoryManager::instance().kmalloc(bitmap_size));
        
        if (!bitmap_data) {
            debug::log(debug::LogLevel::Error, "exFAT", 
                      "Failed to allocate cluster bitmap");
            return false;
        }

        cluster_bitmap_ = Bitmap(bitmap_data, total_clusters_);

        u32 allocation_bitmap_cluster = 2;
        for (u32 i = 0; i < sectors_per_cluster_; i++) {
            if (!read_sector(cluster_heap_start_sector_ + allocation_bitmap_cluster * sectors_per_cluster_ + i,
                            bitmap_data + i * bytes_per_sector_)) {
                mm::VirtualMemoryManager::instance().kfree(bitmap_data);
                debug::log(debug::LogLevel::Error, "exFAT", 
                          "Failed to read allocation bitmap");
                return false;
            }
        }

        debug::log(debug::LogLevel::Info, "exFAT", 
                  "exFAT volume initialized:");
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Sector size: %u bytes", bytes_per_sector_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Cluster size: %u bytes (%u sectors)",
                  bytes_per_cluster_, sectors_per_cluster_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Total clusters: %u", total_clusters_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  FAT start sector: %u", fat_start_sector_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  FAT size: %u sectors", fat_size_sectors_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Cluster heap start: %u", cluster_heap_start_sector_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Root directory cluster: %u", root_dir_cluster_);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Volume flags: 0x%04X", bs_.volume_flags);
        debug::log(debug::LogLevel::Info, "exFAT", 
                  "  Serial number: 0x%08X", bs_.volume_serial_number);

        return true;
    }

    bool exFATVolume::read_sector(u64 sector, void* buffer) {
        return controller_->read(port_index_, partition_start_ + sector, 1, buffer);
    }

    bool exFATVolume::write_sector(u64 sector, const void* buffer) {
        return controller_->write(port_index_, partition_start_ + sector, 1, buffer);
    }

    bool exFATVolume::read_cluster(u32 cluster, void* buffer) {
        if (cluster < 2 || cluster >= total_clusters_ + 2) {
            return false;
        }

        CacheEntry* cached = get_cached_cluster(cluster);
        if (cached) {
            memcpy(buffer, cached->data, bytes_per_cluster_);
            update_cache_statistics(true);
            return true;
        }

        u64 start_sector = cluster_heap_start_sector_ + (cluster - 2) * sectors_per_cluster_;
        u8* current = static_cast<u8*>(buffer);
        
        for (u32 i = 0; i < sectors_per_cluster_; i++) {
            if (!read_sector(start_sector + i, current)) {
                update_cache_statistics(false);
                return false;
            }
            current += bytes_per_sector_;
        }

        put_cached_cluster(cluster, static_cast<const u8*>(buffer));
        update_cache_statistics(false);
        
        return true;
    }

    bool exFATVolume::write_cluster(u32 cluster, const void* buffer) {
        if (cluster < 2 || cluster >= total_clusters_ + 2) {
            return false;
        }

        CacheEntry* cached = get_cached_cluster(cluster);
        if (cached) {
            memcpy(cached->data, buffer, bytes_per_cluster_);
            cached->dirty = true;
            cached->last_access = arch::CPU::read_tsc();
            update_cache_statistics(true);
            return true;
        }

        u64 start_sector = cluster_heap_start_sector_ + (cluster - 2) * sectors_per_cluster_;
        const u8* current = static_cast<const u8*>(buffer);
        
        for (u32 i = 0; i < sectors_per_cluster_; i++) {
            if (!write_sector(start_sector + i, current)) {
                update_cache_statistics(false);
                return false;
            }
            current += bytes_per_sector_;
        }

        put_cached_cluster(cluster, static_cast<const u8*>(buffer));
        update_cache_statistics(false);
        
        return true;
    }

    u32 exFATVolume::read_fat_entry(u32 cluster) {
        if (cluster < 2 || cluster >= total_clusters_ + 2) {
            return 0xFFFFFFFF;
        }

        u64 fat_sector = fat_start_sector_ + (cluster * 4) / bytes_per_sector_;
        u32 fat_offset = (cluster * 4) % bytes_per_sector_;

        u8 sector_buffer[512];
        if (!read_sector(fat_sector, sector_buffer)) {
            return 0xFFFFFFFF;
        }

        u32* fat_entry = reinterpret_cast<u32*>(sector_buffer + fat_offset);
        return *fat_entry;
    }

    bool exFATVolume::write_fat_entry(u32 cluster, u32 value) {
        if (cluster < 2 || cluster >= total_clusters_ + 2) {
            return false;
        }

        u64 fat_sector = fat_start_sector_ + (cluster * 4) / bytes_per_sector_;
        u32 fat_offset = (cluster * 4) % bytes_per_sector_;

        u8 sector_buffer[512];
        if (!read_sector(fat_sector, sector_buffer)) {
            return false;
        }

        u32* fat_entry = reinterpret_cast<u32*>(sector_buffer + fat_offset);
        *fat_entry = value;

        for (u32 i = 0; i < bs_.number_of_fats; i++) {
            if (!write_sector(fat_sector + i * fat_size_sectors_, sector_buffer)) {
                return false;
            }
        }

        return true;
    }

    u32 exFATVolume::allocate_cluster() {
        ScopedLock lock(lock_);

        for (u32 cluster = 2; cluster < total_clusters_ + 2; cluster++) {
            if (!cluster_bitmap_.test(cluster - 2)) {
                cluster_bitmap_.set(cluster - 2, true);

                u32 allocation_bitmap_cluster = 2;
                u32 bitmap_sector_offset = (cluster - 2) / (bytes_per_sector_ * 8);
                u32 bitmap_byte_offset = ((cluster - 2) % (bytes_per_sector_ * 8)) / 8;
                u32 bitmap_bit_offset = (cluster - 2) % 8;

                u8 sector_buffer[512];
                u64 bitmap_sector = cluster_heap_start_sector_ + 
                                   allocation_bitmap_cluster * sectors_per_cluster_ + 
                                   bitmap_sector_offset;

                if (!read_sector(bitmap_sector, sector_buffer)) {
                    cluster_bitmap_.set(cluster - 2, false);
                    return 0;
                }

                sector_buffer[bitmap_byte_offset] |= (1 << bitmap_bit_offset);

                if (!write_sector(bitmap_sector, sector_buffer)) {
                    cluster_bitmap_.set(cluster - 2, false);
                    return 0;
                }

                if (!write_fat_entry(cluster, 0xFFFFFFF7)) {
                    cluster_bitmap_.set(cluster - 2, false);
                    return 0;
                }

                u8* cluster_data = reinterpret_cast<u8*>(
                    mm::VirtualMemoryManager::instance().kmalloc(bytes_per_cluster_));
                
                if (cluster_data) {
                    memset(cluster_data, 0, bytes_per_cluster_);
                    write_cluster(cluster, cluster_data);
                    mm::VirtualMemoryManager::instance().kfree(cluster_data);
                }

                debug::log(debug::LogLevel::Debug, "exFAT",
                          "Allocated cluster %u", cluster);
                
                return cluster;
            }
        }

        return 0;
    }

    bool exFATVolume::free_cluster_chain(u32 first_cluster) {
        ScopedLock lock(lock_);

        u32 current = first_cluster;
        while (current >= 2 && current < total_clusters_ + 2) {
            u32 next = read_fat_entry(current);
            
            cluster_bitmap_.set(current - 2, false);

            u32 allocation_bitmap_cluster = 2;
            u32 bitmap_sector_offset = (current - 2) / (bytes_per_sector_ * 8);
            u32 bitmap_byte_offset = ((current - 2) % (bytes_per_sector_ * 8)) / 8;
            u32 bitmap_bit_offset = (current - 2) % 8;

            u8 sector_buffer[512];
            u64 bitmap_sector = cluster_heap_start_sector_ + 
                               allocation_bitmap_cluster * sectors_per_cluster_ + 
                               bitmap_sector_offset;

            if (read_sector(bitmap_sector, sector_buffer)) {
                sector_buffer[bitmap_byte_offset] &= ~(1 << bitmap_bit_offset);
                write_sector(bitmap_sector, sector_buffer);
            }

            write_fat_entry(current, 0);

            current = next;
        }

        return true;
    }

    u32 exFATVolume::find_next_cluster(u32 current_cluster) {
        if (current_cluster < 2 || current_cluster >= total_clusters_ + 2) {
            return 0xFFFFFFF7;
        }

        u32 fat_entry = read_fat_entry(current_cluster);
        
        if (fat_entry == 0xFFFFFFF7) {
            return 0xFFFFFFF7;
        }

        if (fat_entry >= 0xFFFFFFF8) {
            return 0xFFFFFFFF;
        }

        return fat_entry;
    }

    u32 exFATVolume::cluster_to_sector(u32 cluster) {
        if (cluster < 2) {
            return 0;
        }
        
        return cluster_heap_start_sector_ + (cluster - 2) * sectors_per_cluster_;
    }

    bool exFATVolume::read_cluster_chain(u32 first_cluster, u64 offset, u64 size, void* buffer) {
        if (first_cluster < 2 || first_cluster >= total_clusters_ + 2) {
            return false;
        }

        if (size == 0) {
            return true;
        }

        u32 current_cluster = first_cluster;
        u64 current_offset = 0;
        u8* output = static_cast<u8*>(buffer);

        while (current_cluster < 0xFFFFFFF8 && size > 0) {
            if (current_cluster == 0xFFFFFFF7) {
                return false;
            }

            if (current_cluster >= 0xFFFFFFF8) {
                break;
            }

            u64 cluster_start = current_offset;
            u64 cluster_end = cluster_start + bytes_per_cluster_;

            if (offset < cluster_end && (offset + size) > cluster_start) {
                u64 read_offset = max(offset, cluster_start) - cluster_start;
                u64 read_size = min(size, cluster_end - max(offset, cluster_start));

                u8 cluster_buffer[bytes_per_cluster_];
                if (!read_cluster(current_cluster, cluster_buffer)) {
                    return false;
                }

                memcpy(output, cluster_buffer + read_offset, read_size);
                output += read_size;
                size -= read_size;
                offset += read_size;
            }

            current_offset += bytes_per_cluster_;
            current_cluster = find_next_cluster(current_cluster);
        }

        return size == 0;
    }

    bool exFATVolume::write_cluster_chain(u32 first_cluster, u64 offset, u64 size, const void* buffer) {
        if (first_cluster < 2 || first_cluster >= total_clusters_ + 2) {
            return false;
        }

        if (size == 0) {
            return true;
        }

        u32 current_cluster = first_cluster;
        u64 current_offset = 0;
        const u8* input = static_cast<const u8*>(buffer);

        while (current_cluster < 0xFFFFFFF8 && size > 0) {
            if (current_cluster == 0xFFFFFFF7) {
                return false;
            }

            if (current_cluster >= 0xFFFFFFF8) {
                u32 new_cluster = allocate_cluster();
                if (new_cluster == 0) {
                    return false;
                }

                if (current_cluster == 0xFFFFFFFF) {
                    first_cluster = new_cluster;
                } else {
                    write_fat_entry(current_cluster, new_cluster);
                }

                current_cluster = new_cluster;
            }

            u64 cluster_start = current_offset;
            u64 cluster_end = cluster_start + bytes_per_cluster_;

            if (offset < cluster_end && (offset + size) > cluster_start) {
                u64 write_offset = max(offset, cluster_start) - cluster_start;
                u64 write_size = min(size, cluster_end - max(offset, cluster_start));

                if (write_offset == 0 && write_size == bytes_per_cluster_) {
                    if (!write_cluster(current_cluster, input)) {
                        return false;
                    }
                } else {
                    u8 cluster_buffer[bytes_per_cluster_];
                    if (!read_cluster(current_cluster, cluster_buffer)) {
                        return false;
                    }

                    memcpy(cluster_buffer + write_offset, input, write_size);
                    
                    if (!write_cluster(current_cluster, cluster_buffer)) {
                        return false;
                    }
                }

                input += write_size;
                size -= write_size;
                offset += write_size;
            }

            current_offset += bytes_per_cluster_;
            current_cluster = find_next_cluster(current_cluster);
        }

        while (size > 0) {
            u32 new_cluster = allocate_cluster();
            if (new_cluster == 0) {
                return false;
            }

            if (current_cluster == 0xFFFFFFFF) {
                first_cluster = new_cluster;
            } else {
                write_fat_entry(current_cluster, new_cluster);
            }

            current_cluster = new_cluster;

            u64 write_size = min(size, static_cast<u64>(bytes_per_cluster_));
            
            if (write_size == bytes_per_cluster_) {
                if (!write_cluster(current_cluster, input)) {
                    return false;
                }
            } else {
                u8 cluster_buffer[bytes_per_cluster_];
                memset(cluster_buffer, 0, bytes_per_cluster_);
                memcpy(cluster_buffer, input, write_size);
                
                if (!write_cluster(current_cluster, cluster_buffer)) {
                    return false;
                }
            }

            input += write_size;
            size -= write_size;
            current_cluster = find_next_cluster(current_cluster);
        }

        return true;
    }

    bool exFATVolume::read_directory(u32 cluster, Vector<u8>& buffer) {
        buffer.clear();

        if (cluster < 2 || cluster >= total_clusters_ + 2) {
            return false;
        }

        Vector<u8> cluster_buffer(bytes_per_cluster_);
        u32 current_cluster = cluster;

        while (current_cluster < 0xFFFFFFF8) {
            if (current_cluster == 0xFFFFFFF7) {
                return false;
            }

            if (current_cluster >= 0xFFFFFFF8) {
                break;
            }

            if (!read_cluster(current_cluster, cluster_buffer.data())) {
                return false;
            }

            bool found_end = false;
            for (usize i = 0; i < bytes_per_cluster_; i += 32) {
                u8 entry_type = cluster_buffer[i];
                if (entry_type == 0x00) {
                    found_end = true;
                    break;
                }
            }

            buffer.append(cluster_buffer.data(), cluster_buffer.size());

            if (found_end) {
                break;
            }

            current_cluster = find_next_cluster(current_cluster);
        }

        return true;
    }

    bool exFATVolume::parse_directory(const u8* buffer, usize size, Vector<VFS::DirectoryEntry>& entries) {
        entries.clear();

        for (usize i = 0; i + 32 <= size; i += 32) {
            u8 entry_type = buffer[i];
            
            if (entry_type == 0x00) {
                break;
            }

            if (entry_type == 0x85) {
                if (i + 64 > size) {
                    break;
                }

                const exFATDirectoryEntry* dir_entry = 
                    reinterpret_cast<const exFATDirectoryEntry*>(buffer + i);
                const exFATStreamExtensionEntry* stream_entry = 
                    reinterpret_cast<const exFATStreamExtensionEntry*>(buffer + i + 32);

                u8 secondary_count = dir_entry->secondary_count;
                if (secondary_count < 2) {
                    continue;
                }

                VFS::DirectoryEntry entry;
                entry.type = (dir_entry->file_attributes & 0x10) ? 
                            VFS::EntryType::Directory : VFS::EntryType::File;
                
                entry.size = stream_entry->data_length;
                entry.create_time = convert_timestamp(dir_entry->create_timestamp,
                                                     dir_entry->create_time_10ms,
                                                     dir_entry->create_timezone);
                entry.modify_time = convert_timestamp(dir_entry->last_modified_timestamp,
                                                     dir_entry->last_modified_time_10ms,
                                                     dir_entry->last_modified_timezone);
                entry.access_time = convert_timestamp(dir_entry->last_accessed_timestamp,
                                                     0,
                                                     dir_entry->last_accessed_timezone);

                String name;
                for (u8 j = 0; j < stream_entry->name_length && j < secondary_count - 1; j++) {
                    const exFATFileNameEntry* name_entry = 
                        reinterpret_cast<const exFATFileNameEntry*>(buffer + i + (j + 2) * 32);
                    
                    for (u8 k = 0; k < 15; k++) {
                        u16 ch = name_entry->name_character[k];
                        if (ch == 0) {
                            break;
                        }
                        name.push_back(static_cast<char>(ch & 0xFF));
                    }
                }

                entry.name = name;
                entries.push_back(entry);

                i += (secondary_count * 32) - 32;
            }
        }

        return true;
    }

    bool exFATVolume::find_file_in_directory(u32 directory_cluster, const char* name, FileHandle& file) {
        Vector<u8> buffer;
        if (!read_directory(directory_cluster, buffer)) {
            return false;
        }

        Vector<VFS::DirectoryEntry> entries;
        if (!parse_directory(buffer.data(), buffer.size(), entries)) {
            return false;
        }

        for (const VFS::DirectoryEntry& entry : entries) {
            if (entry.name == name) {
                const u8* buf_ptr = buffer.data();
                for (usize i = 0; i + 32 <= buffer.size(); i += 32) {
                    u8 entry_type = buf_ptr[i];
                    if (entry_type == 0x85) {
                        const exFATDirectoryEntry* dir_entry = 
                            reinterpret_cast<const exFATDirectoryEntry*>(buf_ptr + i);
                        const exFATStreamExtensionEntry* stream_entry = 
                            reinterpret_cast<const exFATStreamExtensionEntry*>(buf_ptr + i + 32);

                        String current_name;
                        u8 secondary_count = dir_entry->secondary_count;
                        
                        for (u8 j = 0; j < stream_entry->name_length && j < secondary_count - 1; j++) {
                            const exFATFileNameEntry* name_entry = 
                                reinterpret_cast<const exFATFileNameEntry*>(buf_ptr + i + (j + 2) * 32);
                            
                            for (u8 k = 0; k < 15; k++) {
                                u16 ch = name_entry->name_character[k];
                                if (ch == 0) {
                                    break;
                                }
                                current_name.push_back(static_cast<char>(ch & 0xFF));
                            }
                        }

                        if (current_name == name) {
                            file.first_cluster = stream_entry->first_cluster;
                            file.file_size = stream_entry->data_length;
                            file.current_offset = 0;
                            file.current_cluster = stream_entry->first_cluster;
                            file.cluster_offset = 0;
                            file.attributes = dir_entry->file_attributes;
                            file.create_time = convert_timestamp(dir_entry->create_timestamp,
                                                               dir_entry->create_time_10ms,
                                                               dir_entry->create_timezone);
                            file.modify_time = convert_timestamp(dir_entry->last_modified_timestamp,
                                                               dir_entry->last_modified_time_10ms,
                                                               dir_entry->last_modified_timezone);
                            file.access_time = convert_timestamp(dir_entry->last_accessed_timestamp,
                                                               0,
                                                               dir_entry->last_accessed_timezone);
                            return true;
                        }

                        i += (secondary_count * 32) - 32;
                    }
                }
            }
        }

        return false;
    }

    bool exFATVolume::create_directory_entry(u32 directory_cluster, const VFS::DirectoryEntry& entry, FileHandle& file) {
        Vector<u8> buffer;
        if (!read_directory(directory_cluster, buffer)) {
            return false;
        }

        usize free_slot = 0;
        bool found_free = false;
        
        for (usize i = 0; i + 32 <= buffer.size(); i += 32) {
            u8 entry_type = buffer[i];
            if (entry_type == 0x00) {
                free_slot = i;
                found_free = true;
                break;
            }
            
            if (entry_type == 0x85) {
                const exFATDirectoryEntry* dir_entry = 
                    reinterpret_cast<const exFATDirectoryEntry*>(buffer.data() + i);
                i += (dir_entry->secondary_count * 32) - 32;
            }
        }

        if (!found_free) {
            free_slot = buffer.size();
            buffer.resize(buffer.size() + bytes_per_cluster_, 0);
        }

        u8 name_length = static_cast<u8>(entry.name.length());
        u8 secondary_count = 2 + (name_length + 14) / 15;

        if (free_slot + secondary_count * 32 > buffer.size()) {
            buffer.resize(free_slot + secondary_count * 32, 0);
        }

        u64 current_time = arch::CPU::read_tsc();
        exFATDateTime now;
        now.year = 2024;
        now.month = 1;
        now.day = 1;
        now.hour = 0;
        now.minute = 0;
        now.second = 0;
        now.milliseconds = 0;

        exFATDirectoryEntry* dir_entry = 
            reinterpret_cast<exFATDirectoryEntry*>(buffer.data() + free_slot);
        dir_entry->entry_type = 0x85;
        dir_entry->secondary_count = secondary_count;
        dir_entry->file_attributes = (entry.type == VFS::EntryType::Directory) ? 0x10 : 0x20;
        dir_entry->create_timestamp = convert_to_timestamp(now);
        dir_entry->last_modified_timestamp = convert_to_timestamp(now);
        dir_entry->last_accessed_timestamp = convert_to_timestamp(now);
        dir_entry->create_time_10ms = 0;
        dir_entry->last_modified_time_10ms = 0;
        dir_entry->create_timezone = 0;
        dir_entry->last_modified_timezone = 0;
        dir_entry->last_accessed_timezone = 0;

        exFATStreamExtensionEntry* stream_entry = 
            reinterpret_cast<exFATStreamExtensionEntry*>(buffer.data() + free_slot + 32);
        stream_entry->entry_type = 0xC0;
        stream_entry->secondary_count = secondary_count - 1;
        stream_entry->name_length = name_length;
        stream_entry->name_hash = calculate_name_hash(entry.name.c_str());
        stream_entry->valid_data_length = 0;
        stream_entry->first_cluster = file.first_cluster;
        stream_entry->data_length = file.file_size;

        const char* name_ptr = entry.name.c_str();
        for (u8 i = 0; i < secondary_count - 1; i++) {
            exFATFileNameEntry* name_entry = 
                reinterpret_cast<exFATFileNameEntry*>(buffer.data() + free_slot + (i + 2) * 32);
            name_entry->entry_type = 0xC1;
            name_entry->flags = 0;

            for (u8 j = 0; j < 15; j++) {
                u8 idx = i * 15 + j;
                if (idx < name_length) {
                    name_entry->name_character[j] = name_ptr[idx];
                } else {
                    name_entry->name_character[j] = 0;
                }
            }
        }

        dir_entry->set_checksum = calculate_checksum(buffer.data() + free_slot, secondary_count * 32);

        if (!write_cluster_chain(directory_cluster, free_slot, secondary_count * 32, buffer.data() + free_slot)) {
            return false;
        }

        file.create_time = now;
        file.modify_time = now;
        file.access_time = now;

        return true;
    }

    bool exFATVolume::delete_directory_entry(u32 directory_cluster, const char* name) {
        Vector<u8> buffer;
        if (!read_directory(directory_cluster, buffer)) {
            return false;
        }

        for (usize i = 0; i + 32 <= buffer.size(); i += 32) {
            u8 entry_type = buffer[i];
            
            if (entry_type == 0x00) {
                break;
            }

            if (entry_type == 0x85) {
                const exFATDirectoryEntry* dir_entry = 
                    reinterpret_cast<const exFATDirectoryEntry*>(buffer.data() + i);
                const exFATStreamExtensionEntry* stream_entry = 
                    reinterpret_cast<const exFATStreamExtensionEntry*>(buffer.data() + i + 32);

                String current_name;
                u8 secondary_count = dir_entry->secondary_count;
                
                for (u8 j = 0; j < stream_entry->name_length && j < secondary_count - 1; j++) {
                    const exFATFileNameEntry* name_entry = 
                        reinterpret_cast<const exFATFileNameEntry*>(buffer.data() + i + (j + 2) * 32);
                    
                    for (u8 k = 0; k < 15; k++) {
                        u16 ch = name_entry->name_character[k];
                        if (ch == 0) {
                            break;
                        }
                        current_name.push_back(static_cast<char>(ch & 0xFF));
                    }
                }

                if (current_name == name) {
                    memset(buffer.data() + i, 0, secondary_count * 32);
                    
                    if (!write_cluster_chain(directory_cluster, i, secondary_count * 32, buffer.data() + i)) {
                        return false;
                    }

                    if (stream_entry->first_cluster >= 2) {
                        free_cluster_chain(stream_entry->first_cluster);
                    }

                    return true;
                }

                i += (secondary_count * 32) - 32;
            }
        }

        return false;
    }

    u16 exFATVolume::calculate_name_hash(const char* name) {
        u16 hash = 0;
        while (*name) {
            hash = ((hash << 15) | (hash >> 1)) + static_cast<u8>(*name);
            name++;
        }
        return hash;
    }

    u16 exFATVolume::calculate_checksum(const u8* entries, usize count) {
        u16 checksum = 0;
        for (usize i = 0; i < count; i++) {
            if (i == 2 || i == 3) {
                checksum = ((checksum << 15) | (checksum >> 1)) + 0;
            } else {
                checksum = ((checksum << 15) | (checksum >> 1)) + entries[i];
            }
        }
        return checksum;
    }

    exFATDateTime exFATVolume::convert_timestamp(u32 timestamp, u8 subseconds, u8 timezone) {
        exFATDateTime dt;
        
        u16 date = timestamp >> 16;
        u16 time = timestamp & 0xFFFF;
        
        dt.year = ((date >> 9) & 0x7F) + 1980;
        dt.month = (date >> 5) & 0x0F;
        dt.day = date & 0x1F;
        dt.hour = (time >> 11) & 0x1F;
        dt.minute = (time >> 5) & 0x3F;
        dt.second = (time & 0x1F) * 2;
        dt.milliseconds = subseconds * 10;
        
        return dt;
    }

    u32 exFATVolume::convert_to_timestamp(const exFATDateTime& dt) {
        u16 date = ((dt.year - 1980) << 9) | (dt.month << 5) | dt.day;
        u16 time = (dt.hour << 11) | (dt.minute << 5) | (dt.second / 2);
        
        return (date << 16) | time;
    }

    void exFATVolume::update_cache_statistics(bool hit) {
        if (hit) {
            cache_hits_++;
        } else {
            cache_misses_++;
        }
    }

    void exFATVolume::flush_cache() {
        ScopedLock lock(lock_);
        
        for (CacheEntry& entry : cache_) {
            if (entry.dirty) {
                u64 start_sector = cluster_heap_start_sector_ + (entry.cluster - 2) * sectors_per_cluster_;
                const u8* current = entry.data;
                
                for (u32 i = 0; i < sectors_per_cluster_; i++) {
                    write_sector(start_sector + i, current);
                    current += bytes_per_sector_;
                }
                
                entry.dirty = false;
            }
        }
    }

    exFATVolume::CacheEntry* exFATVolume::get_cached_cluster(u32 cluster) {
        ScopedLock lock(lock_);
        
        for (CacheEntry& entry : cache_) {
            if (entry.cluster == cluster) {
                entry.last_access = arch::CPU::read_tsc();
                return &entry;
            }
        }
        
        return nullptr;
    }

    void exFATVolume::put_cached_cluster(u32 cluster, const u8* data) {
        ScopedLock lock(lock_);
        
        if (cache_.size() >= 64) {
            auto oldest = cache_.begin();
            for (auto it = cache_.begin(); it != cache_.end(); ++it) {
                if (it->last_access < oldest->last_access) {
                    oldest = it;
                }
            }
            
            if (oldest->dirty) {
                u64 start_sector = cluster_heap_start_sector_ + (oldest->cluster - 2) * sectors_per_cluster_;
                const u8* current = oldest->data;
                
                for (u32 i = 0; i < sectors_per_cluster_; i++) {
                    write_sector(start_sector + i, current);
                    current += bytes_per_sector_;
                }
            }
            
            mm::VirtualMemoryManager::instance().kfree(oldest->data);
            cache_.erase(oldest);
        }
        
        CacheEntry entry;
        entry.cluster = cluster;
        entry.data = reinterpret_cast<u8*>(
            mm::VirtualMemoryManager::instance().kmalloc(bytes_per_cluster_));
        
        if (!entry.data) {
            return;
        }
        
        memcpy(entry.data, data, bytes_per_cluster_);
        entry.dirty = false;
        entry.last_access = arch::CPU::read_tsc();
        
        cache_.push_back(entry);
    }

    VFS::FileHandle* exFATVolume::open(const char* path, u32 flags) {
        ScopedLock lock(lock_);
        
        if (path == nullptr || path[0] != '/') {
            return nullptr;
        }

        const char* name = path + 1;
        if (name[0] == '\0') {
            return nullptr;
        }

        FileHandle* file = new FileHandle();
        if (!find_file_in_directory(root_dir_cluster_, name, *file)) {
            if (flags & VFS::OpenFlags::Create) {
                file->first_cluster = allocate_cluster();
                if (file->first_cluster == 0) {
                    delete file;
                    return nullptr;
                }

                file->file_size = 0;
                file->current_offset = 0;
                file->current_cluster = file->first_cluster;
                file->cluster_offset = 0;
                file->attributes = (flags & VFS::OpenFlags::Directory) ? 0x10 : 0x20;

                VFS::DirectoryEntry entry;
                entry.name = name;
                entry.type = (flags & VFS::OpenFlags::Directory) ? 
                            VFS::EntryType::Directory : VFS::EntryType::File;
                entry.size = 0;

                if (!create_directory_entry(root_dir_cluster_, entry, *file)) {
                    free_cluster_chain(file->first_cluster);
                    delete file;
                    return nullptr;
                }
            } else {
                delete file;
                return nullptr;
            }
        } else {
            if ((flags & VFS::OpenFlags::Create) && (flags & VFS::OpenFlags::Exclusive)) {
                delete file;
                return nullptr;
            }

            if ((flags & VFS::OpenFlags::Directory) && !(file->attributes & 0x10)) {
                delete file;
                return nullptr;
            }
        }

        return reinterpret_cast<VFS::FileHandle*>(file);
    }

    bool exFATVolume::close(VFS::FileHandle* handle) {
        if (!handle) {
            return false;
        }

        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        delete file;
        return true;
    }

    usize exFATVolume::read(VFS::FileHandle* handle, void* buffer, usize size) {
        if (!handle || !buffer) {
            return 0;
        }

        ScopedLock lock(lock_);
        
        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        
        if (file->attributes & 0x10) {
            return 0;
        }

        if (file->current_offset >= file->file_size) {
            return 0;
        }

        usize to_read = min(size, static_cast<usize>(file->file_size - file->current_offset));
        
        if (!read_cluster_chain(file->first_cluster, file->current_offset, to_read, buffer)) {
            return 0;
        }

        file->current_offset += to_read;
        
        if (file->current_offset >= file->file_size) {
            file->current_cluster = 0xFFFFFFFF;
            file->cluster_offset = 0;
        } else {
            file->current_cluster = file->first_cluster;
            file->cluster_offset = file->current_offset;
            
            while (file->cluster_offset >= bytes_per_cluster_) {
                file->current_cluster = find_next_cluster(file->current_cluster);
                file->cluster_offset -= bytes_per_cluster_;
            }
        }

        return to_read;
    }

    usize exFATVolume::write(VFS::FileHandle* handle, const void* buffer, usize size) {
        if (!handle || !buffer) {
            return 0;
        }

        ScopedLock lock(lock_);
        
        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        
        if (file->attributes & 0x10) {
            return 0;
        }

        if (file->current_offset + size > file->file_size) {
            if (!truncate(handle, file->current_offset + size)) {
                return 0;
            }
        }

        if (!write_cluster_chain(file->first_cluster, file->current_offset, size, buffer)) {
            return 0;
        }

        file->current_offset += size;
        
        if (file->current_offset >= file->file_size) {
            file->current_cluster = 0xFFFFFFFF;
            file->cluster_offset = 0;
        } else {
            file->current_cluster = file->first_cluster;
            file->cluster_offset = file->current_offset;
            
            while (file->cluster_offset >= bytes_per_cluster_) {
                file->current_cluster = find_next_cluster(file->current_cluster);
                file->cluster_offset -= bytes_per_cluster_;
            }
        }

        return size;
    }

    bool exFATVolume::seek(VFS::FileHandle* handle, i64 offset, VFS::SeekMode mode) {
        if (!handle) {
            return false;
        }

        ScopedLock lock(lock_);
        
        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        
        u64 new_offset;
        switch (mode) {
            case VFS::SeekMode::Set:
                new_offset = offset;
                break;
            case VFS::SeekMode::Current:
                new_offset = file->current_offset + offset;
                break;
            case VFS::SeekMode::End:
                new_offset = file->file_size + offset;
                break;
            default:
                return false;
        }

        if (new_offset > file->file_size) {
            return false;
        }

        file->current_offset = new_offset;
        
        if (new_offset == file->file_size) {
            file->current_cluster = 0xFFFFFFFF;
            file->cluster_offset = 0;
        } else {
            file->current_cluster = file->first_cluster;
            file->cluster_offset = new_offset;
            
            while (file->cluster_offset >= bytes_per_cluster_) {
                file->current_cluster = find_next_cluster(file->current_cluster);
                file->cluster_offset -= bytes_per_cluster_;
            }
        }

        return true;
    }

    u64 exFATVolume::tell(VFS::FileHandle* handle) {
        if (!handle) {
            return 0;
        }

        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        return file->current_offset;
    }

    bool exFATVolume::truncate(VFS::FileHandle* handle, u64 size) {
        if (!handle) {
            return false;
        }

        ScopedLock lock(lock_);
        
        FileHandle* file = reinterpret_cast<FileHandle*>(handle);
        
        if (file->attributes & 0x10) {
            return false;
        }

        if (size == file->file_size) {
            return true;
        }

        if (size < file->file_size) {
            u32 clusters_needed = (size + bytes_per_cluster_ - 1) / bytes_per_cluster_;
            u32 current_cluster = file->first_cluster;
            u32 prev_cluster = 0;
            
            for (u32 i = 0; i < clusters_needed; i++) {
                if (current_cluster >= 0xFFFFFFF8) {
                    break;
                }
                prev_cluster = current_cluster;
                current_cluster = find_next_cluster(current_cluster);
            }
            
            if (prev_cluster != 0) {
                write_fat_entry(prev_cluster, 0xFFFFFFFF);
            }
            
            if (current_cluster >= 2 && current_cluster < total_clusters_ + 2) {
                free_cluster_chain(current_cluster);
            }
        } else {
            u64 additional = size - file->file_size;
            u64 current_pos = file->file_size;
            
            while (additional > 0) {
                u64 to_write = min(additional, static_cast<u64>(bytes_per_cluster_));
                
                u8 zero_buffer[bytes_per_cluster_];
                memset(zero_buffer, 0, bytes_per_cluster_);
                
                if (!write_cluster_chain(file->first_cluster, current_pos, to_write, zero_buffer)) {
                    return false;
                }
                
                current_pos += to_write;
                additional -= to_write;
            }
        }

        file->file_size = size;
        
        Vector<u8> buffer;
        if (!read_directory(root_dir_cluster_, buffer)) {
            return false;
        }

        for (usize i = 0; i + 32 <= buffer.size(); i += 32) {
            u8 entry_type = buffer[i];
            
            if (entry_type == 0x00) {
                break;
            }

            if (entry_type == 0x85) {
                exFATDirectoryEntry* dir_entry = 
                    reinterpret_cast<exFATDirectoryEntry*>(buffer.data() + i);
                exFATStreamExtensionEntry* stream_entry = 
                    reinterpret_cast<exFATStreamExtensionEntry*>(buffer.data() + i + 32);

                String current_name;
                u8 secondary_count = dir_entry->secondary_count;
                
                for (u8 j = 0; j < stream_entry->name_length && j < secondary_count - 1; j++) {
                    const exFATFileNameEntry* name_entry = 
                        reinterpret_cast<const exFATFileNameEntry*>(buffer.data() + i + (j + 2) * 32);
                    
                    for (u8 k = 0; k < 15; k++) {
                        u16 ch = name_entry->name_character[k];
                        if (ch == 0) {
                            break;
                        }
                        current_name.push_back(static_cast<char>(ch & 0xFF));
                    }
                }

                if (current_name == file->name) {
                    stream_entry->data_length = size;
                    dir_entry->set_checksum = calculate_checksum(buffer.data() + i, secondary_count * 32);
                    
                    if (!write_cluster_chain(root_dir_cluster_, i, secondary_count * 32, buffer.data() + i)) {
                        return false;
                    }
                    
                    break;
                }

                i += (secondary_count * 32) - 32;
            }
        }

        return true;
    }

    VFS::DirectoryHandle* exFATVolume::opendir(const char* path) {
        ScopedLock lock(lock_);
        
        if (path == nullptr || path[0] != '/') {
            return nullptr;
        }

        const char* name = path + 1;
        
        FileHandle file;
        if (name[0] != '\0') {
            if (!find_file_in_directory(root_dir_cluster_, name, file)) {
                return nullptr;
            }
            
            if (!(file.attributes & 0x10)) {
                return nullptr;
            }
        } else {
            file.first_cluster = root_dir_cluster_;
            file.attributes = 0x10;
        }

        DirectoryHandle* dir = new DirectoryHandle();
        dir->cluster = file.first_cluster;
        dir->offset = 0;
        
        if (!read_directory(dir->cluster, dir->buffer)) {
            delete dir;
            return nullptr;
        }

        return reinterpret_cast<VFS::DirectoryHandle*>(dir);
    }

    bool exFATVolume::readdir(VFS::DirectoryHandle* handle, VFS::DirectoryEntry& entry) {
        if (!handle) {
            return false;
        }

        ScopedLock lock(lock_);
        
        DirectoryHandle* dir = reinterpret_cast<DirectoryHandle*>(handle);
        
        Vector<VFS::DirectoryEntry> entries;
        if (!parse_directory(dir->buffer.data(), dir->buffer.size(), entries)) {
            return false;
        }

        if (dir->offset >= entries.size()) {
            return false;
        }

        entry = entries[dir->offset];
        dir->offset++;
        
        return true;
    }

    bool exFATVolume::closedir(VFS::DirectoryHandle* handle) {
        if (!handle) {
            return false;
        }

        DirectoryHandle* dir = reinterpret_cast<DirectoryHandle*>(handle);
        delete dir;
        return true;
    }

    bool exFATVolume::mkdir(const char* path) {
        ScopedLock lock(lock_);
        
        VFS::FileHandle* handle = open(path, VFS::OpenFlags::Create | VFS::OpenFlags::Directory);
        if (!handle) {
            return false;
        }
        
        return close(handle);
    }

    bool exFATVolume::rmdir(const char* path) {
        ScopedLock lock(lock_);
        
        if (path == nullptr || path[0] != '/') {
            return false;
        }

        const char* name = path + 1;
        if (name[0] == '\0') {
            return false;
        }

        return delete_directory_entry(root_dir_cluster_, name);
    }

    bool exFATVolume::unlink(const char* path) {
        return rmdir(path);
    }

    bool exFATVolume::rename(const char* old_path, const char* new_path) {
        ScopedLock lock(lock_);
        
        if (!old_path || !new_path || old_path[0] != '/' || new_path[0] != '/') {
            return false;
        }

        const char* old_name = old_path + 1;
        const char* new_name = new_path + 1;
        
        if (old_name[0] == '\0' || new_name[0] == '\0') {
            return false;
        }

        FileHandle file;
        if (!find_file_in_directory(root_dir_cluster_, old_name, file)) {
            return false;
        }

        Vector<u8> buffer;
        if (!read_directory(root_dir_cluster_, buffer)) {
            return false;
        }

        for (usize i = 0; i + 32 <= buffer.size(); i += 32) {
            u8 entry_type = buffer[i];
            
            if (entry_type == 0x00) {
                break;
            }

            if (entry_type == 0x85) {
                exFATDirectoryEntry* dir_entry = 
                    reinterpret_cast<exFATDirectoryEntry*>(buffer.data() + i);
                exFATStreamExtensionEntry* stream_entry = 
                    reinterpret_cast<exFATStreamExtensionEntry*>(buffer.data() + i + 32);

                String current_name;
                u8 secondary_count = dir_entry->secondary_count;
                
                for (u8 j = 0; j < stream_entry->name_length && j < secondary_count - 1; j++) {
                    const exFATFileNameEntry* name_entry = 
                        reinterpret_cast<const exFATFileNameEntry*>(buffer.data() + i + (j + 2) * 32);
                    
                    for (u8 k = 0; k < 15; k++) {
                        u16 ch = name_entry->name_character[k];
                        if (ch == 0) {
                            break;
                        }
                        current_name.push_back(static_cast<char>(ch & 0xFF));
                    }
                }

                if (current_name == old_name) {
                    usize name_entries = secondary_count - 2;
                    usize new_name_length = strlen(new_name);
                    usize new_name_entries = (new_name_length + 14) / 15;
                    
                    if (new_name_entries != name_entries) {
                        return false;
                    }

                    for (u8 j = 0; j < new_name_entries; j++) {
                        exFATFileNameEntry* name_entry = 
                            reinterpret_cast<exFATFileNameEntry*>(buffer.data() + i + (j + 2) * 32);
                        
                        for (u8 k = 0; k < 15; k++) {
                            usize idx = j * 15 + k;
                            if (idx < new_name_length) {
                                name_entry->name_character[k] = new_name[idx];
                            } else {
                                name_entry->name_character[k] = 0;
                            }
                        }
                    }

                    stream_entry->name_length = static_cast<u8>(new_name_length);
                    stream_entry->name_hash = calculate_name_hash(new_name);
                    dir_entry->set_checksum = calculate_checksum(buffer.data() + i, secondary_count * 32);
                    
                    if (!write_cluster_chain(root_dir_cluster_, i, secondary_count * 32, buffer.data() + i)) {
                        return false;
                    }
                    
                    return true;
                }

                i += (secondary_count * 32) - 32;
            }
        }

        return false;
    }

    bool exFATVolume::stat(const char* path, VFS::Stat& stat) {
        ScopedLock lock(lock_);
        
        if (path == nullptr || path[0] != '/') {
            return false;
        }

        const char* name = path + 1;
        
        FileHandle file;
        if (name[0] == '\0') {
            stat.type = VFS::EntryType::Directory;
            stat.size = 0;
            stat.blocks = 0;
            stat.block_size = bytes_per_cluster_;
            return true;
        }

        if (!find_file_in_directory(root_dir_cluster_, name, file)) {
            return false;
        }

        stat.type = (file.attributes & 0x10) ? VFS::EntryType::Directory : VFS::EntryType::File;
        stat.size = file.file_size;
        stat.blocks = (file.file_size + bytes_per_cluster_ - 1) / bytes_per_cluster_;
        stat.block_size = bytes_per_cluster_;
        stat.create_time = file.create_time;
        stat.modify_time = file.modify_time;
        stat.access_time = file.access_time;
        
        return true;
    }

    bool exFATVolume::chmod(const char* path, u32 mode) {
        return true;
    }

    bool exFATVolume::utime(const char* path, const VFS::Time& times) {
        return true;
    }

    u64 exFATVolume::get_total_space() const {
        return static_cast<u64>(total_clusters_) * bytes_per_cluster_;
    }

    u64 exFATVolume::get_free_space() const {
        ScopedLock lock(lock_);
        
        u32 free_clusters = 0;
        for (u32 i = 0; i < total_clusters_; i++) {
            if (!cluster_bitmap_.test(i)) {
                free_clusters++;
            }
        }
        
        return static_cast<u64>(free_clusters) * bytes_per_cluster_;
    }

    u64 exFATVolume::get_used_space() const {
        return get_total_space() - get_free_space();
    }

    void exFATVolume::sync() {
        flush_cache();
    }

    void exFATVolume::dump_info() const {
        debug::log(debug::LogLevel::Info, "exFAT", "Volume Information:");
        debug::log(debug::LogLevel::Info, "exFAT", "  Total space: %llu MB", get_total_space() / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "exFAT", "  Free space: %llu MB", get_free_space() / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "exFAT", "  Used space: %llu MB", get_used_space() / (1024 * 1024));
        debug::log(debug::LogLevel::Info, "exFAT", "  Cache hits: %llu", cache_hits_);
        debug::log(debug::LogLevel::Info, "exFAT", "  Cache misses: %llu", cache_misses_);
        debug::log(debug::LogLevel::Info, "exFAT", "  Cache hit rate: %.2f%%",
                  cache_hits_ + cache_misses_ > 0 ?
                  (100.0 * cache_hits_ / (cache_hits_ + cache_misses_)) : 0.0);
    }

    bool exFATVolume::detect(drivers::AHCIController* controller, u32 port_index, u64 partition_start) {
        exFATBootSector bs;
        
        if (!controller->read(port_index, partition_start, 1, &bs)) {
            return false;
        }

        if (bs.boot_signature != 0xAA55) {
            return false;
        }

        return memcmp(bs.file_system_name, "EXFAT   ", 8) == 0;
    }

    exFATFileSystem::exFATFileSystem() {
        debug::log(debug::LogLevel::Info, "exFAT", "exFAT filesystem driver created");
    }

    exFATFileSystem::~exFATFileSystem() {
        ScopedLock lock(lock_);
        
        for (auto& pair : volumes_) {
            delete pair.second;
        }
        volumes_.clear();
    }

    bool exFATFileSystem::init() {
        debug::log(debug::LogLevel::Info, "exFAT", "Initializing exFAT filesystem");
        detect_volumes();
        return true;
    }

    bool exFATFileSystem::mount(const char* device, const char* mount_point, u32 flags) {
        return true;
    }

    bool exFATFileSystem::unmount(const char* mount_point) {
        return true;
    }

    VFS::Volume* exFATFileSystem::get_volume(const char* mount_point) {
        ScopedLock lock(lock_);
        
        auto it = volumes_.find(mount_point);
        if (it != volumes_.end()) {
            return it->second;
        }
        
        return nullptr;
    }

    void exFATFileSystem::detect_volumes() {
        ScopedLock lock(lock_);
        
        for (u32 i = 0; i < drivers::AHCIManager::get_controller_count(); i++) {
            drivers::AHCIController* controller = drivers::AHCIManager::get_controller(i);
            if (!controller) {
                continue;
            }
            
            for (u32 port = 0; port < controller->get_port_count(); port++) {
                const drivers::AHCIController::PortInfo* port_info = controller->get_port_info(port);
                if (!port_info || !port_info->initialized) {
                    continue;
                }
                
                for (u64 offset = 0; offset < port_info->sector_count; offset += 2048) {
                    if (exFATVolume::detect(controller, port, offset)) {
                        String mount_point = String::format("disk%u-%u", i, port);
                        
                        exFATVolume* volume = new exFATVolume(controller, port, offset);
                        if (volume->init()) {
                            volumes_[mount_point] = volume;
                            debug::log(debug::LogLevel::Success, "exFAT",
                                      "Mounted exFAT volume at %s", mount_point.c_str());
                            volume->dump_info();
                        } else {
                            delete volume;
                        }
                        
                        break;
                    }
                }
            }
        }
    }

    void exFATFileSystem::dump_volumes() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "exFAT", "Mounted exFAT volumes:");
        for (const auto& pair : volumes_) {
            debug::log(debug::LogLevel::Info, "exFAT", "  %s", pair.first.c_str());
        }
    }

    exFATFileSystem& exFATFileSystem::instance() {
        static exFATFileSystem instance;
        return instance;
    }
}
