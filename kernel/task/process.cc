#include <nanokoton/task/process.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/task/scheduler.hpp>
#include <nanokoton/mm/virtual.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/algorithm.hpp>

namespace nk::task {
    Thread::Thread(Process* process, u64 entry_point, usize stack_size)
        : id_(reinterpret_cast<u64>(this)),
          stack_size_(align_up(stack_size, PAGE_SIZE)),
          state_(ThreadState::Created),
          entry_point_(entry_point),
          process_(process),
          sleep_until_(0),
          tls_base_(nullptr),
          tls_size_(0) {
        
        stack_ = reinterpret_cast<u64*>(
            mm::VirtualMemoryManager::instance().kmalloc_aligned(stack_size_, PAGE_SIZE));
        
        if (!stack_) {
            debug::log(debug::LogLevel::Error, "PROC", 
                      "Failed to allocate stack for thread");
            return;
        }
        
        registers_ = reinterpret_cast<RegisterState*>(
            mm::VirtualMemoryManager::instance().kmalloc(sizeof(RegisterState)));
        
        if (!registers_) {
            mm::VirtualMemoryManager::instance().kfree(stack_);
            stack_ = nullptr;
            debug::log(debug::LogLevel::Error, "PROC", 
                      "Failed to allocate registers for thread");
            return;
        }
        
        memset(registers_, 0, sizeof(RegisterState));
        
        registers_->rip = entry_point_;
        registers_->rsp = reinterpret_cast<u64>(stack_) + stack_size_ - 128;
        registers_->cs = 0x08;
        registers_->ss = 0x10;
        registers_->rflags = 0x202;
        
        tls_size_ = 4096;
        tls_base_ = mm::VirtualMemoryManager::instance().kmalloc_aligned(tls_size_, PAGE_SIZE);
        
        if (tls_base_) {
            memset(tls_base_, 0, tls_size_);
        }
        
        state_ = ThreadState::Ready;
        
        debug::log(debug::LogLevel::Debug, "PROC",
                  "Created thread %llu in process %llu, entry: 0x%016llX, stack: 0x%016llX",
                  id_, process->get_pid(), entry_point_, stack_);
    }

    Thread::~Thread() {
        if (stack_) {
            mm::VirtualMemoryManager::instance().kfree(stack_);
        }
        
        if (registers_) {
            mm::VirtualMemoryManager::instance().kfree(registers_);
        }
        
        if (tls_base_) {
            mm::VirtualMemoryManager::instance().kfree(tls_base_);
        }
    }

    bool Thread::should_wake_up(u64 current_time) const {
        return is_sleeping() && current_time >= sleep_until_;
    }

    void Thread::save_context(RegisterState* regs) {
        if (registers_ && regs) {
            *registers_ = *regs;
        }
    }

    void Thread::restore_context(RegisterState* regs) {
        if (registers_ && regs) {
            *regs = *registers_;
        }
    }

    void Thread::dump_state() const {
        debug::log(debug::LogLevel::Info, "THREAD", "Thread %llu:", id_);
        debug::log(debug::LogLevel::Info, "THREAD", "  State: %u", static_cast<u32>(state_));
        debug::log(debug::LogLevel::Info, "THREAD", "  Process: %llu", process_->get_pid());
        debug::log(debug::LogLevel::Info, "THREAD", "  Entry: 0x%016llX", entry_point_);
        debug::log(debug::LogLevel::Info, "THREAD", "  Stack: 0x%016llX (%llu bytes)",
                  stack_, stack_size_);
        if (registers_) {
            debug::log(debug::LogLevel::Info, "THREAD", "  RIP: 0x%016llX, RSP: 0x%016llX",
                      registers_->rip, registers_->rsp);
        }
    }

    Process::Process(u64 pid, u64 parent_pid, const String& name)
        : pid_(pid),
          parent_pid_(parent_pid),
          name_(name),
          state_(ProcessState::Created),
          exit_code_(0),
          address_space_(nullptr),
          main_thread_(nullptr),
          creation_time_(arch::CPU::read_tsc()),
          termination_time_(0) {
        
        memset(&limits_, 0, sizeof(limits_));
        memset(&statistics_, 0, sizeof(statistics_));
        
        limits_.memory_limit = 256 * 1024 * 1024;
        limits_.threads_limit = 64;
        limits_.open_files = 256;
        
        address_space_ = mm::VirtualMemoryManager::instance().create_address_space();
        
        if (!address_space_) {
            debug::log(debug::LogLevel::Error, "PROC",
                      "Failed to create address space for process %llu", pid_);
            return;
        }
        
        main_thread_ = create_thread(0, 8192);
        if (!main_thread_) {
            mm::VirtualMemoryManager::instance().destroy_address_space(address_space_);
            address_space_ = nullptr;
            debug::log(debug::LogLevel::Error, "PROC",
                      "Failed to create main thread for process %llu", pid_);
            return;
        }
        
        state_ = ProcessState::Ready;
        
        debug::log(debug::LogLevel::Info, "PROC",
                  "Created process %llu (parent: %llu, name: '%s')",
                  pid_, parent_pid_, name_.c_str());
    }

    Process::~Process() {
        if (address_space_) {
            mm::VirtualMemoryManager::instance().destroy_address_space(address_space_);
        }
        
        for (Thread* thread : threads_) {
            delete thread;
        }
        threads_.clear();
        
        for (auto& pair : open_files_) {
            if (pair.second) {
                // TODO: Close file handle
            }
        }
        open_files_.clear();
        
        debug::log(debug::LogLevel::Info, "PROC",
                  "Destroyed process %llu", pid_);
    }

    Thread* Process::create_thread(u64 entry_point, usize stack_size) {
        ScopedLock lock(lock_);
        
        if (threads_.size() >= limits_.threads_limit) {
            debug::log(debug::LogLevel::Error, "PROC",
                      "Thread limit reached for process %llu", pid_);
            return nullptr;
        }
        
        Thread* thread = new Thread(this, entry_point, stack_size);
        if (!thread || !thread->get_registers()) {
            delete thread;
            return nullptr;
        }
        
        threads_.push_back(thread);
        
        if (main_thread_ == nullptr) {
            main_thread_ = thread;
        }
        
        return thread;
    }

    bool Process::destroy_thread(u64 thread_id) {
        ScopedLock lock(lock_);
        
        for (usize i = 0; i < threads_.size(); i++) {
            if (threads_[i]->get_id() == thread_id) {
                if (threads_[i] == main_thread_) {
                    main_thread_ = nullptr;
                }
                
                delete threads_[i];
                threads_.erase(i);
                return true;
            }
        }
        
        return false;
    }

    Thread* Process::get_thread(u64 thread_id) {
        ScopedLock lock(lock_);
        
        for (Thread* thread : threads_) {
            if (thread->get_id() == thread_id) {
                return thread;
            }
        }
        
        return nullptr;
    }

    bool Process::add_open_file(u32 fd, void* file) {
        ScopedLock lock(lock_);
        
        if (open_files_.size() >= limits_.open_files) {
            return false;
        }
        
        open_files_[fd] = file;
        return true;
    }

    bool Process::remove_open_file(u32 fd) {
        ScopedLock lock(lock_);
        
        auto it = open_files_.find(fd);
        if (it == open_files_.end()) {
            return false;
        }
        
        open_files_.erase(it);
        return true;
    }

    void* Process::get_open_file(u32 fd) const {
        ScopedLock lock(lock_);
        
        auto it = open_files_.find(fd);
        if (it == open_files_.end()) {
            return nullptr;
        }
        
        return it->second;
    }

    void Process::set_environment(const String& key, const String& value) {
        ScopedLock lock(lock_);
        environment_[key] = value;
    }

    String Process::get_environment(const String& key) const {
        ScopedLock lock(lock_);
        
        auto it = environment_.find(key);
        if (it == environment_.end()) {
            return String();
        }
        
        return it->second;
    }

    void Process::update_statistics(u64 cpu_time_delta, u64 memory_used) {
        ScopedLock lock(lock_);
        
        statistics_.cpu_time_used += cpu_time_delta;
        statistics_.memory_used = memory_used;
        statistics_.context_switches++;
    }

    void Process::dump_state() const {
        debug::log(debug::LogLevel::Info, "PROCESS", "Process %llu:", pid_);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Name: '%s'", name_.c_str());
        debug::log(debug::LogLevel::Info, "PROCESS", "  Parent: %llu", parent_pid_);
        debug::log(debug::LogLevel::Info, "PROCESS", "  State: %u", static_cast<u32>(state_));
        debug::log(debug::LogLevel::Info, "PROCESS", "  Exit Code: %llu", exit_code_);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Threads: %llu", threads_.size());
        debug::log(debug::LogLevel::Info, "PROCESS", "  Open Files: %llu", open_files_.size());
        debug::log(debug::LogLevel::Info, "PROCESS", "  CPU Time: %llu", statistics_.cpu_time_used);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Memory Used: %llu", statistics_.memory_used);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Context Switches: %llu", statistics_.context_switches);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Page Faults: %llu", statistics_.page_faults);
        debug::log(debug::LogLevel::Info, "PROCESS", "  Creation Time: %llu", creation_time_);
        if (termination_time_ > 0) {
            debug::log(debug::LogLevel::Info, "PROCESS", "  Termination Time: %llu", termination_time_);
        }
    }

    ProcessManager::ProcessManager() 
        : next_pid_(1),
          kernel_process_(nullptr) {
        debug::log(debug::LogLevel::Info, "PROCMGR", "Process Manager created");
    }

    ProcessManager::~ProcessManager() {
        ScopedLock lock(lock_);
        
        for (auto& pair : processes_) {
            delete pair.second;
        }
        processes_.clear();
    }

    bool ProcessManager::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "PROCMGR", "Initializing Process Manager");
        
        kernel_process_ = create_process("kernel", 0);
        if (!kernel_process_) {
            debug::log(debug::LogLevel::Error, "PROCMGR", 
                      "Failed to create kernel process");
            return false;
        }
        
        kernel_process_->state_ = ProcessState::Running;
        
        debug::log(debug::LogLevel::Info, "PROCMGR", 
                  "Process Manager initialized with kernel process %llu",
                  kernel_process_->get_pid());
        
        return true;
    }

    u64 ProcessManager::allocate_pid() {
        while (processes_.find(next_pid_) != processes_.end()) {
            next_pid_++;
            if (next_pid_ == 0) {
                next_pid_ = 1;
            }
        }
        
        return next_pid_++;
    }

    void ProcessManager::cleanup_zombies() {
        Vector<u64> to_remove;
        
        for (const auto& pair : processes_) {
            if (pair.second->is_zombie()) {
                to_remove.push_back(pair.first);
            }
        }
        
        for (u64 pid : to_remove) {
            auto it = processes_.find(pid);
            if (it != processes_.end()) {
                delete it->second;
                processes_.erase(it);
            }
        }
    }

    Process* ProcessManager::create_process(const String& name, u64 parent_pid) {
        ScopedLock lock(lock_);
        
        u64 pid = allocate_pid();
        Process* process = new Process(pid, parent_pid, name);
        
        if (!process || !process->get_address_space()) {
            delete process;
            return nullptr;
        }
        
        processes_[pid] = process;
        
        debug::log(debug::LogLevel::Info, "PROCMGR",
                  "Created process %llu: '%s'", pid, name.c_str());
        
        return process;
    }

    bool ProcessManager::destroy_process(u64 pid) {
        ScopedLock lock(lock_);
        
        auto it = processes_.find(pid);
        if (it == processes_.end()) {
            return false;
        }
        
        Process* process = it->second;
        process->state_ = ProcessState::Zombie;
        process->termination_time_ = arch::CPU::read_tsc();
        
        cleanup_zombies();
        
        return true;
    }

    Process* ProcessManager::get_process(u64 pid) {
        ScopedLock lock(lock_);
        
        auto it = processes_.find(pid);
        if (it == processes_.end()) {
            return nullptr;
        }
        
        return it->second;
    }

    Process* ProcessManager::get_current_process() {
        Scheduler& scheduler = Scheduler::instance();
        Thread* thread = scheduler.get_current_thread();
        
        if (!thread) {
            return kernel_process_;
        }
        
        return thread->get_process();
    }

    void ProcessManager::set_current_process(Process* process) {
        // Process switching is handled by the scheduler
    }

    void ProcessManager::update_process_statistics(u64 pid, u64 cpu_time_delta, u64 memory_used) {
        ScopedLock lock(lock_);
        
        auto it = processes_.find(pid);
        if (it != processes_.end()) {
            it->second->update_statistics(cpu_time_delta, memory_used);
        }
    }

    void ProcessManager::dump_processes() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "PROCMGR", "Processes (%llu):", processes_.size());
        for (const auto& pair : processes_) {
            const Process* process = pair.second;
            debug::log(debug::LogLevel::Info, "PROCMGR",
                      "  PID: %llu, Name: '%s', State: %u, Parent: %llu",
                      pair.first, process->get_name().c_str(),
                      static_cast<u32>(process->get_state()),
                      process->get_parent_pid());
        }
    }

    void ProcessManager::dump_process(u64 pid) const {
        ScopedLock lock(lock_);
        
        auto it = processes_.find(pid);
        if (it != processes_.end()) {
            it->second->dump_state();
        }
    }

    ProcessManager& ProcessManager::instance() {
        static ProcessManager instance;
        return instance;
    }
}
