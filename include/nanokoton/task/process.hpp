#ifndef NANOKOTON_PROCESS_HPP
#define NANOKOTON_PROCESS_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/lib/hashmap.hpp>

namespace nk::task {
    enum class ProcessState {
        Created,
        Ready,
        Running,
        Blocked,
        Sleeping,
        Zombie,
        Dead
    };

    enum class ThreadState {
        Created,
        Ready,
        Running,
        Blocked,
        Sleeping,
        Dead
    };

    struct ProcessLimits {
        u64 cpu_time;
        u64 memory_limit;
        u64 open_files;
        u64 threads_limit;
    };

    struct ProcessStatistics {
        u64 cpu_time_used;
        u64 memory_used;
        u64 context_switches;
        u64 page_faults;
        u64 io_operations;
        u64 signals_received;
    };

    class Thread {
    private:
        u64 id_;
        u64* stack_;
        usize stack_size_;
        ThreadState state_;
        u64 entry_point_;
        RegisterState* registers_;
        Process* process_;
        u64 sleep_until_;
        
        void* tls_base_;
        usize tls_size_;
        
    public:
        Thread(Process* process, u64 entry_point, usize stack_size = 8192);
        ~Thread();
        
        u64 get_id() const { return id_; }
        ThreadState get_state() const { return state_; }
        Process* get_process() const { return process_; }
        
        void set_state(ThreadState state) { state_ = state; }
        void set_sleep_until(u64 timestamp) { sleep_until_ = timestamp; }
        
        RegisterState* get_registers() { return registers_; }
        const RegisterState* get_registers() const { return registers_; }
        
        void* get_stack_top() const { return stack_ + stack_size_; }
        usize get_stack_size() const { return stack_size_; }
        
        void* get_tls_base() const { return tls_base_; }
        usize get_tls_size() const { return tls_size_; }
        
        bool is_sleeping() const { return state_ == ThreadState::Sleeping; }
        bool should_wake_up(u64 current_time) const;
        
        void save_context(RegisterState* regs);
        void restore_context(RegisterState* regs);
        
        void dump_state() const;
    };

    class Process {
    private:
        u64 pid_;
        u64 parent_pid_;
        String name_;
        ProcessState state_;
        u64 exit_code_;
        
        mm::VirtualMemoryManager::AddressSpace* address_space_;
        Vector<Thread*> threads_;
        Thread* main_thread_;
        
        ProcessLimits limits_;
        ProcessStatistics statistics_;
        
        HashMap<u32, void*> open_files_;
        HashMap<String, String> environment_;
        Vector<String> arguments_;
        
        u64 creation_time_;
        u64 termination_time_;
        
        Mutex lock_;
        
    public:
        Process(u64 pid, u64 parent_pid, const String& name);
        ~Process();
        
        u64 get_pid() const { return pid_; }
        u64 get_parent_pid() const { return parent_pid_; }
        const String& get_name() const { return name_; }
        ProcessState get_state() const { return state_; }
        
        mm::VirtualMemoryManager::AddressSpace* get_address_space() { return address_space_; }
        const mm::VirtualMemoryManager::AddressSpace* get_address_space() const { return address_space_; }
        
        Thread* create_thread(u64 entry_point, usize stack_size = 8192);
        bool destroy_thread(u64 thread_id);
        Thread* get_thread(u64 thread_id);
        Vector<Thread*> get_threads() const { return threads_; }
        usize get_thread_count() const { return threads_.size(); }
        
        void set_state(ProcessState state) { state_ = state; }
        void set_exit_code(u64 code) { exit_code_ = code; }
        u64 get_exit_code() const { return exit_code_; }
        
        const ProcessLimits& get_limits() const { return limits_; }
        void set_limits(const ProcessLimits& limits) { limits_ = limits; }
        
        ProcessStatistics& get_statistics() { return statistics_; }
        const ProcessStatistics& get_statistics() const { return statistics_; }
        
        bool add_open_file(u32 fd, void* file);
        bool remove_open_file(u32 fd);
        void* get_open_file(u32 fd) const;
        
        void set_environment(const String& key, const String& value);
        String get_environment(const String& key) const;
        const HashMap<String, String>& get_environment() const { return environment_; }
        
        void set_arguments(const Vector<String>& args) { arguments_ = args; }
        const Vector<String>& get_arguments() const { return arguments_; }
        
        u64 get_creation_time() const { return creation_time_; }
        u64 get_termination_time() const { return termination_time_; }
        void set_termination_time(u64 time) { termination_time_ = time; }
        
        void update_statistics(u64 cpu_time_delta, u64 memory_used);
        
        bool is_zombie() const { return state_ == ProcessState::Zombie; }
        bool is_dead() const { return state_ == ProcessState::Dead; }
        
        void dump_state() const;
    };

    class ProcessManager {
    private:
        HashMap<u64, Process*> processes_;
        u64 next_pid_;
        Process* kernel_process_;
        
        Mutex lock_;
        
        u64 allocate_pid();
        void cleanup_zombies();
        
    public:
        ProcessManager();
        ~ProcessManager();
        
        bool init();
        
        Process* create_process(const String& name, u64 parent_pid = 0);
        bool destroy_process(u64 pid);
        Process* get_process(u64 pid);
        
        Process* get_current_process();
        void set_current_process(Process* process);
        
        u64 get_process_count() const { return processes_.size(); }
        
        void update_process_statistics(u64 pid, u64 cpu_time_delta, u64 memory_used);
        
        void dump_processes() const;
        void dump_process(u64 pid) const;
        
        static ProcessManager& instance();
    };
}
#endif
