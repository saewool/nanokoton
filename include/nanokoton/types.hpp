#ifndef NANOKOTON_TYPES_HPP
#define NANOKOTON_TYPES_HPP

#include <cstdint>
#include <cstddef>
#include <cstdbool>

namespace nk {
    using u8 = std::uint8_t;
    using u16 = std::uint16_t;
    using u32 = std::uint32_t;
    using u64 = std::uint64_t;
    using usize = std::size_t;

    using i8 = std::int8_t;
    using i16 = std::int16_t;
    using i32 = std::int32_t;
    using i64 = std::int64_t;
    using isize = std::ptrdiff_t;

    using f32 = float;
    using f64 = double;

    using phys_addr = u64;
    using virt_addr = u64;

    struct RegisterState {
        u64 r15, r14, r13, r12, r11, r10, r9, r8;
        u64 rdi, rsi, rbp, rbx, rdx, rcx, rax;
        u64 interrupt_vector, error_code;
        u64 rip, cs, rflags, rsp, ss;
    };

    enum class MemoryType : u32 {
        Free = 0,
        Reserved = 1,
        ACPIReclaimable = 2,
        ACPINVS = 3,
        BadMemory = 4,
        BootloaderReclaimable = 5,
        KernelAndModules = 6,
        Framebuffer = 7
    };

    struct MemoryMapEntry {
        phys_addr base_address;
        usize region_length;
        MemoryType region_type;
        u32 extended_attributes;
    };

    struct BootInfo {
        u64 magic_number;
        u64 version;
        u64 memory_map_address;
        u64 memory_map_entry_count;
        u64 framebuffer_address;
        u64 framebuffer_width;
        u64 framebuffer_height;
        u64 framebuffer_pitch;
        u64 framebuffer_bpp;
        u64 rsdp_address;
        u64 kernel_start;
        u64 kernel_end;
        u64 initrd_start;
        u64 initrd_end;
    };

    #define PACKED __attribute__((packed))
    #define ALIGNED(x) __attribute__((aligned(x)))
    #define NO_RETURN __attribute__((noreturn))
    #define ALWAYS_INLINE __attribute__((always_inline))
    #define NO_INLINE __attribute__((noinline))
    #define SECTION(x) __attribute__((section(x)))
    #define USED __attribute__((used))
    #define UNUSED __attribute__((unused))
    #define WEAK __attribute__((weak))
    #define PURE __attribute__((pure))
    #define CONST __attribute__((const))
    #define FORCE_INLINE inline __attribute__((always_inline))
    #define FORCE_NOINLINE __attribute__((noinline))

    template<typename T>
    class Optional {
    private:
        bool has_value_;
        union {
            T value_;
            u8 dummy_[sizeof(T)];
        };

    public:
        Optional() : has_value_(false) {}
        Optional(const T& value) : has_value_(true), value_(value) {}
        Optional(T&& value) : has_value_(true), value_(std::move(value)) {}

        bool has_value() const { return has_value_; }
        T& value() { return value_; }
        const T& value() const { return value_; }
        T value_or(const T& default_value) const { return has_value_ ? value_ : default_value; }

        void reset() { has_value_ = false; }
    };

    template<typename T, usize N>
    class Array {
    private:
        T data_[N];

    public:
        constexpr usize size() const { return N; }
        constexpr bool empty() const { return N == 0; }

        T& operator[](usize index) { return data_[index]; }
        const T& operator[](usize index) const { return data_[index]; }

        T* begin() { return data_; }
        const T* begin() const { return data_; }
        T* end() { return data_ + N; }
        const T* end() const { return data_ + N; }
    };

    class NonCopyable {
    protected:
        NonCopyable() = default;
        ~NonCopyable() = default;
        NonCopyable(const NonCopyable&) = delete;
        NonCopyable& operator=(const NonCopyable&) = delete;
    };

    class NonMovable {
    protected:
        NonMovable() = default;
        ~NonMovable() = default;
        NonMovable(NonMovable&&) = delete;
        NonMovable& operator=(NonMovable&&) = delete;
    };
}

#endif
