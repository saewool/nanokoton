#include <nanokoton/core/debug.hpp>
#include <nanokoton/drivers/serial.hpp>
#include <nanokoton/drivers/vga.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/stdio.hpp>

namespace nk {
    namespace debug {
        static LogLevel current_log_level = LogLevel::Info;
        static bool serial_logging_enabled = true;
        static bool vga_logging_enabled = true;

        void init() {
            current_log_level = LogLevel::Debug;
            serial_logging_enabled = true;
            vga_logging_enabled = true;
        }

        void set_log_level(LogLevel level) {
            current_log_level = level;
        }

        void enable_serial_logging(bool enable) {
            serial_logging_enabled = enable;
        }

        void enable_vga_logging(bool enable) {
            vga_logging_enabled = enable;
        }

        void log(LogLevel level, const char* component, const char* format, ...) {
            if (level < current_log_level) {
                return;
            }

            const char* level_strings[] = {
                "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
            };

            const char* level_string = level_strings[static_cast<int>(level)];
            char buffer[512];

            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);

            char message[1024];
            snprintf(message, sizeof(message), "[%s] [%s] %s\n", level_string, component, buffer);

            if (serial_logging_enabled) {
                drivers::Serial::write_string(message);
            }

            if (vga_logging_enabled) {
                drivers::VGA::write_string(message);
            }

            if (level == LogLevel::Fatal) {
                Kernel::panic(buffer);
            }
        }

        void dump_registers(const RegisterState* regs) {
            log(LogLevel::Error, "CPU", "Register Dump:");
            log(LogLevel::Error, "CPU", "RAX: 0x%016llX RBX: 0x%016llX RCX: 0x%016llX RDX: 0x%016llX",
                regs->rax, regs->rbx, regs->rcx, regs->rdx);
            log(LogLevel::Error, "CPU", "RSI: 0x%016llX RDI: 0x%016llX RBP: 0x%016llX RSP: 0x%016llX",
                regs->rsi, regs->rdi, regs->rbp, regs->rsp);
            log(LogLevel::Error, "CPU", "R8:  0x%016llX R9:  0x%016llX R10: 0x%016llX R11: 0x%016llX",
                regs->r8, regs->r9, regs->r10, regs->r11);
            log(LogLevel::Error, "CPU", "R12: 0x%016llX R13: 0x%016llX R14: 0x%016llX R15: 0x%016llX",
                regs->r12, regs->r13, regs->r14, regs->r15);
            log(LogLevel::Error, "CPU", "RIP: 0x%016llX RFLAGS: 0x%016llX",
                regs->rip, regs->rflags);
            log(LogLevel::Error, "CPU", "CS: 0x%04X SS: 0x%04X Interrupt: %u Error: 0x%016llX",
                regs->cs, regs->ss, regs->interrupt_vector, regs->error_code);
        }

        void dump_memory(const void* address, usize size) {
            const u8* data = static_cast<const u8*>(address);
            
            log(LogLevel::Debug, "MEM", "Memory dump at 0x%016llX, size: %llu bytes",
                reinterpret_cast<u64>(address), size);
            
            for (usize i = 0; i < size; i += 16) {
                char hex_buffer[128];
                char ascii_buffer[32];
                usize offset = 0;
                
                offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset,
                                 "0x%016llX: ", reinterpret_cast<u64>(data + i));
                
                for (usize j = 0; j < 16; j++) {
                    if (i + j < size) {
                        offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset,
                                         "%02X ", data[i + j]);
                        ascii_buffer[j] = (data[i + j] >= 32 && data[i + j] < 127) ? 
                                         data[i + j] : '.';
                    } else {
                        offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset, "   ");
                        ascii_buffer[j] = ' ';
                    }
                }
                
                ascii_buffer[16] = '\0';
                log(LogLevel::Debug, "MEM", "%s |%s|", hex_buffer, ascii_buffer);
            }
        }

        void dump_stack_trace(usize max_frames) {
            u64* rbp;
            asm volatile("mov %%rbp, %0" : "=r"(rbp));
            
            log(LogLevel::Debug, "STACK", "Stack trace:");
            
            for (usize frame = 0; frame < max_frames && rbp; frame++) {
                u64 rip = rbp[1];
                rbp = reinterpret_cast<u64*>(rbp[0]);
                
                if (!rip) {
                    break;
                }
                
                log(LogLevel::Debug, "STACK", "  #%llu: 0x%016llX", frame, rip);
            }
        }

        void breakpoint() {
            asm volatile("int3");
        }

        void assert_failed(const char* expression, const char* file, int line) {
            log(LogLevel::Fatal, "ASSERT", "Assertion failed: %s, file %s, line %d",
                expression, file, line);
        }
    }
}
