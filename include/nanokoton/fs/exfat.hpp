#ifndef NANOKOTON_EXFAT_HPP
#define NANOKOTON_EXFAT_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/fs/vfs.hpp>
#include <nanokoton/drivers/ahci.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/hashmap.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/lib/bitmap.hpp>

namespace nk::fs {
    struct PACKED exFATBootSector {
        u8 jump_boot[3];
        u8 file_system_name[8];
        u8 must_be_zero[53];
        u64 partition_offset;
        u64 volume_length;
        u32 fat_offset;
        u32 fat_length;
        u32 cluster_heap_offset;
        u32 cluster_count;
        u32 first_cluster_of_root_directory;
        u32 volume_serial_number;
        u16 file_system_revision;
        u16 volume_flags;
        u8 bytes_per_sector_shift;
        u8 sectors_per_cluster_shift;
        u8 number_of_fats;
        u8 drive_select;
        u8 percent_in_use;
        u8 reserved[7];
        u8 boot_code[390];
        u16 boot_signature;
    };

    struct PACKED exFATDirectoryEntry {
        u8 entry_type;
        u8 secondary_count;
        u16 set_checksum;
        u16 file_attributes;
        u16 reserved1;
        u32 create_timestamp;
        u32 last_modified_timestamp;
        u32 last_accessed_timestamp;
        u8 create_time_10ms;
        u8 last_modified_time_10ms;
        u8 create_timezone;
        u8 last_modified_timezone;
        u8 last_accessed_timezone;
        u8 reserved2[7];
    };

    struct PACKED exFATStreamExtensionEntry {
        u8 entry_type;
        u8 secondary_count;
        u16 flags;
        u8 reserved1;
        u8 name_length;
        u16 name_hash;
        u8 reserved2[2];
        u64 valid_data_length;
        u8 reserved3[4];
        u32 first_cluster;
        u64 data_length;
    };

    struct PACKED exFATFileNameEntry {
        u8 entry_type;
        u8 flags;
        u16 name_character[15];
    };

    enum class exFATEntryType : u8 {
        EndOfDirectory = 0x00,
        AllocationBitmap = 0x01,
        UpCaseTable = 0x02,
        VolumeLabel = 0x03,
        FileDirectory = 0x85,
        StreamExtension = 0xC0,
        FileName = 0xC1,
        VendorExtension = 0xE0
    };

    enum class exFATAttribute : u16 {
        ReadOnly = 0x0001,
        Hidden = 0x0002,
        System = 0x0004,
        Directory = 0x0010,
        Archive = 0x0020
    };

    struct exFATDateTime {
        u16 year;
        u8 month;
        u8 day;
        u8 hour;
        u8 minute;
        u8 second;
        u8 milliseconds;
    };

    class exFATVolume : public VFS::Volume {
    private:
        struct CacheEntry {
            u64 cluster;
            u8* data;
            bool dirty;
            u64 last_access;
        };

        struct FileHandle {
            u32 first_cluster;
            u64 file_size;
            u64 current_offset;
            u32 current_cluster;
            u64 cluster_offset;
            u16 attributes;
            exFATDateTime create_time;
            exFATDateTime modify_time;
            exFATDateTime access_time;
        };

        struct DirectoryHandle {
            u32 cluster;
            u64 offset;
            Vector<u8> buffer;
        };

        exFATBootSector bs_;
        drivers::AHCIController* controller_;
        u32 port_index_;
        u64 partition_start_;
        
        u32 bytes_per_sector_;
        u32 sectors_per_cluster_;
        u32 bytes_per_cluster_;
        u32 total_clusters_;
        u32 fat_start_sector_;
        u32 fat_size_sectors_;
        u32 cluster_heap_start_sector_;
        u32 root_dir_cluster_;
        
        Bitmap cluster_bitmap_;
        Vector<CacheEntry> cache_;
        HashMap<u32, u32> cluster_chain_cache_;
        Mutex lock_;
        u64 cache_hits_;
        u64 cache_misses_;
        
        bool read_sector(u64 sector, void* buffer);
        bool write_sector(u64 sector, const void* buffer);
        bool read_cluster(u32 cluster, void* buffer);
        bool write_cluster(u32 cluster, const void* buffer);
        
        u32 read_fat_entry(u32 cluster);
        bool write_fat_entry(u32 cluster, u32 value);
        u32 allocate_cluster();
        bool free_cluster_chain(u32 first_cluster);
        u32 find_next_cluster(u32 current_cluster);
        u32 cluster_to_sector(u32 cluster);
        
        bool read_cluster_chain(u32 first_cluster, u64 offset, u64 size, void* buffer);
        bool write_cluster_chain(u32 first_cluster, u64 offset, u64 size, const void* buffer);
        
        bool read_directory(u32 cluster, Vector<u8>& buffer);
        bool parse_directory(const u8* buffer, usize size, Vector<VFS::DirectoryEntry>& entries);
        bool find_file_in_directory(u32 directory_cluster, const char* name, FileHandle& file);
        bool create_directory_entry(u32 directory_cluster, const VFS::DirectoryEntry& entry, FileHandle& file);
        bool delete_directory_entry(u32 directory_cluster, const char* name);
        
        u16 calculate_name_hash(const char* name);
        u16 calculate_checksum(const u8* entries, usize count);
        
        exFATDateTime convert_timestamp(u32 timestamp, u8 subseconds, u8 timezone);
        u32 convert_to_timestamp(const exFATDateTime& dt);
        
        void update_cache_statistics(bool hit);
        void flush_cache();
        CacheEntry* get_cached_cluster(u32 cluster);
        void put_cached_cluster(u32 cluster, const u8* data);
        
    public:
        exFATVolume(drivers::AHCIController* controller, u32 port_index, u64 partition_start);
        ~exFATVolume();
        
        bool init() override;
        const char* get_name() const override { return "exFAT"; }
        
        VFS::FileHandle* open(const char* path, u32 flags) override;
        bool close(VFS::FileHandle* handle) override;
        usize read(VFS::FileHandle* handle, void* buffer, usize size) override;
        usize write(VFS::FileHandle* handle, const void* buffer, usize size) override;
        bool seek(VFS::FileHandle* handle, i64 offset, VFS::SeekMode mode) override;
        u64 tell(VFS::FileHandle* handle) override;
        bool truncate(VFS::FileHandle* handle, u64 size) override;
        
        VFS::DirectoryHandle* opendir(const char* path) override;
        bool readdir(VFS::DirectoryHandle* handle, VFS::DirectoryEntry& entry) override;
        bool closedir(VFS::DirectoryHandle* handle) override;
        
        bool mkdir(const char* path) override;
        bool rmdir(const char* path) override;
        bool unlink(const char* path) override;
        bool rename(const char* old_path, const char* new_path) override;
        
        bool stat(const char* path, VFS::Stat& stat) override;
        bool chmod(const char* path, u32 mode) override;
        bool utime(const char* path, const VFS::Time& times) override;
        
        u64 get_total_space() const override;
        u64 get_free_space() const override;
        u64 get_used_space() const override;
        
        void sync() override;
        void dump_info() const override;
        
        bool format(u64 total_sectors);
        bool check_and_repair();
        
        static bool detect(drivers::AHCIController* controller, u32 port_index, u64 partition_start);
    };
    
    class exFATFileSystem : public VFS::FileSystem {
    private:
        HashMap<String, exFATVolume*> volumes_;
        Mutex lock_;
        
    public:
        exFATFileSystem();
        ~exFATFileSystem();
        
        bool init() override;
        bool mount(const char* device, const char* mount_point, u32 flags) override;
        bool unmount(const char* mount_point) override;
        VFS::Volume* get_volume(const char* mount_point) override;
        
        void detect_volumes();
        void dump_volumes() const;
        
        static exFATFileSystem& instance();
    };
}
#endif
