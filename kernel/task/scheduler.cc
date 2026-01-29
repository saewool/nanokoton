#include <nanokoton/task/scheduler.hpp>
#include <nanokoton/core/debug.hpp>
#include <nanokoton/task/process.hpp>
#include <nanokoton/arch/cpu.hpp>
#include <nanokoton/lib/string.hpp>
#include <nanokoton/lib/algorithm.hpp>

namespace nk::task {
    Scheduler::Scheduler()
        : idle_thread_(nullptr),
          current_thread_(nullptr),
          idle_process_(nullptr),
          policy_(SchedulingPolicy::RoundRobin),
          time_slice_default_(10000),
          last_schedule_time_(0),
          timer_ticks_(0),
          cpu_count_(1) {
        
        memset(&statistics_, 0, sizeof(statistics_));
        
        for (u32 i = 0; i < 4; i++) {
            RunQueue queue;
            queue.current_index = 0;
            queue.time_slice = time_slice_default_;
            queue.priority_level = i;
            run_queues_.push_back(queue);
        }
        
        cpu_affinity_ = Bitmap(nullptr, 64);
    }

    Scheduler::~Scheduler() {
        if (idle_thread_) {
            delete idle_thread_;
        }
        
        if (idle_process_) {
            delete idle_process_;
        }
        
        for (RunQueue& queue : run_queues_) {
            for (Thread* thread : queue.threads) {
                delete thread;
            }
            queue.threads.clear();
        }
        run_queues_.clear();
    }

    bool Scheduler::init() {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "SCHED", "Initializing Scheduler");
        
        ProcessManager& proc_mgr = ProcessManager::instance();
        idle_process_ = proc_mgr.create_process("idle", 0);
        
        if (!idle_process_) {
            debug::log(debug::LogLevel::Error, "SCHED", 
                      "Failed to create idle process");
            return false;
        }
        
        idle_process_->state_ = ProcessState::Running;
        
        idle_thread_ = idle_process_->create_thread(0, 4096);
        if (!idle_thread_) {
            delete idle_process_;
            idle_process_ = nullptr;
            debug::log(debug::LogLevel::Error, "SCHED",
                      "Failed to create idle thread");
            return false;
        }
        
        idle_thread_->set_state(ThreadState::Running);
        current_thread_ = idle_thread_;
        
        last_schedule_time_ = arch::CPU::read_tsc();
        statistics_.last_switch_time = last_schedule_time_;
        
        debug::log(debug::LogLevel::Info, "SCHED",
                  "Scheduler initialized with %u priority levels",
                  run_queues_.size());
        
        return true;
    }

    void Scheduler::start() {
        debug::log(debug::LogLevel::Info, "SCHED", "Scheduler started");
    }

    void Scheduler::add_thread(Thread* thread) {
        ScopedLock lock(lock_);
        
        if (!validate_thread(thread)) {
            debug::log(debug::LogLevel::Error, "SCHED",
                      "Invalid thread %llu", thread->get_id());
            return;
        }
        
        u32 priority = calculate_priority(thread);
        if (priority >= run_queues_.size()) {
            priority = run_queues_.size() - 1;
        }
        
        thread->set_state(ThreadState::Ready);
        run_queues_[priority].threads.push_back(thread);
        
        debug::log(debug::LogLevel::Debug, "SCHED",
                  "Added thread %llu to priority queue %u",
                  thread->get_id(), priority);
    }

    void Scheduler::remove_thread(Thread* thread) {
        ScopedLock lock(lock_);
        
        for (RunQueue& queue : run_queues_) {
            for (usize i = 0; i < queue.threads.size(); i++) {
                if (queue.threads[i] == thread) {
                    queue.threads.erase(i);
                    
                    if (thread == current_thread_) {
                        current_thread_ = idle_thread_;
                    }
                    
                    debug::log(debug::LogLevel::Debug, "SCHED",
                              "Removed thread %llu from scheduler",
                              thread->get_id());
                    return;
                }
            }
        }
    }

    void Scheduler::yield() {
        ScopedLock lock(lock_);
        
        if (current_thread_ == idle_thread_) {
            return;
        }
        
        Thread* next_thread = select_next_thread();
        if (next_thread && next_thread != current_thread_) {
            switch_to_thread(next_thread);
        }
    }

    void Scheduler::sleep(u64 milliseconds) {
        ScopedLock lock(lock_);
        
        if (current_thread_ == idle_thread_) {
            return;
        }
        
        u64 wake_time = arch::CPU::read_tsc() + milliseconds * 1000000;
        current_thread_->set_sleep_until(wake_time);
        current_thread_->set_state(ThreadState::Sleeping);
        
        Thread* next_thread = select_next_thread();
        if (next_thread) {
            switch_to_thread(next_thread);
        }
    }

    void Scheduler::wake_up(Thread* thread) {
        ScopedLock lock(lock_);
        
        if (!thread || thread->get_state() != ThreadState::Sleeping) {
            return;
        }
        
        thread->set_state(ThreadState::Ready);
        add_thread(thread);
        
        debug::log(debug::LogLevel::Debug, "SCHED",
                  "Woke up thread %llu", thread->get_id());
    }

    Thread* Scheduler::select_next_thread() {
        u64 current_time = arch::CPU::read_tsc();
        
        cleanup_dead_threads();
        
        for (usize priority = 0; priority < run_queues_.size(); priority++) {
            RunQueue& queue = run_queues_[priority];
            
            if (queue.threads.empty()) {
                continue;
            }
            
            queue.current_index %= queue.threads.size();
            
            for (usize i = 0; i < queue.threads.size(); i++) {
                Thread* thread = queue.threads[queue.current_index];
                queue.current_index = (queue.current_index + 1) % queue.threads.size();
                
                if (validate_thread(thread)) {
                    if (thread->get_state() == ThreadState::Ready) {
                        return thread;
                    }
                    
                    if (thread->get_state() == ThreadState::Sleeping) {
                        if (thread->should_wake_up(current_time)) {
                            thread->set_state(ThreadState::Ready);
                            return thread;
                        }
                    }
                }
            }
        }
        
        return idle_thread_;
    }

    void Scheduler::update_thread_priorities() {
        u64 current_time = arch::CPU::read_tsc();
        
        for (RunQueue& queue : run_queues_) {
            for (Thread* thread : queue.threads) {
                if (thread->get_state() == ThreadState::Running) {
                    u64 time_slice = calculate_time_slice(thread);
                    if (current_time - last_schedule_time_ > time_slice) {
                        u32 new_priority = calculate_priority(thread);
                        if (new_priority != queue.priority_level) {
                            remove_thread(thread);
                            add_thread(thread);
                        }
                    }
                }
            }
        }
    }

    void Scheduler::handle_timer_tick() {
        ScopedLock lock(lock_);
        
        timer_ticks_++;
        
        u64 current_time = arch::CPU::read_tsc();
        
        if (current_thread_ != idle_thread_) {
            u64 time_slice = calculate_time_slice(current_thread_);
            if (current_time - last_schedule_time_ > time_slice) {
                Thread* next_thread = select_next_thread();
                if (next_thread && next_thread != current_thread_) {
                    switch_to_thread(next_thread);
                }
            }
        }
        
        for (RunQueue& queue : run_queues_) {
            for (Thread* thread : queue.threads) {
                if (thread->is_sleeping() && thread->should_wake_up(current_time)) {
                    thread->set_state(ThreadState::Ready);
                }
            }
        }
        
        cleanup_dead_threads();
    }

    bool Scheduler::validate_thread(Thread* thread) {
        if (!thread) {
            return false;
        }
        
        if (thread->get_state() == ThreadState::Dead) {
            return false;
        }
        
        Process* process = thread->get_process();
        if (!process || process->is_dead() || process->is_zombie()) {
            return false;
        }
        
        return true;
    }

    void Scheduler::cleanup_dead_threads() {
        for (RunQueue& queue : run_queues_) {
            for (usize i = 0; i < queue.threads.size(); i++) {
                Thread* thread = queue.threads[i];
                if (thread->get_state() == ThreadState::Dead) {
                    if (thread == current_thread_) {
                        current_thread_ = idle_thread_;
                    }
                    delete thread;
                    queue.threads.erase(i);
                    i--;
                }
            }
        }
    }

    u64 Scheduler::calculate_time_slice(Thread* thread) {
        Process* process = thread->get_process();
        if (!process) {
            return time_slice_default_;
        }
        
        u64 base_slice = time_slice_default_;
        
        switch (policy_) {
            case SchedulingPolicy::RoundRobin:
                return base_slice;
                
            case SchedulingPolicy::Priority:
                return base_slice * (4 - thread->get_priority()) / 4;
                
            case SchedulingPolicy::RealTime:
                return base_slice * 2;
                
            case SchedulingPolicy::Fair:
                return base_slice * process->get_statistics().cpu_time_used / 
                       (statistics_.total_cpu_time + 1);
                
            default:
                return base_slice;
        }
    }

    u32 Scheduler::calculate_priority(Thread* thread) {
        Process* process = thread->get_process();
        if (!process) {
            return 0;
        }
        
        u32 priority = 0;
        
        switch (policy_) {
            case SchedulingPolicy::RoundRobin:
                priority = 1;
                break;
                
            case SchedulingPolicy::Priority:
                priority = thread->get_priority();
                break;
                
            case SchedulingPolicy::RealTime:
                priority = 0;
                break;
                
            case SchedulingPolicy::Fair:
                priority = 2;
                break;
                
            default:
                priority = 1;
        }
        
        if (priority >= run_queues_.size()) {
            priority = run_queues_.size() - 1;
        }
        
        return priority;
    }

    void Scheduler::switch_to_thread(Thread* thread) {
        if (!thread || thread == current_thread_) {
            return;
        }
        
        Thread* old_thread = current_thread_;
        current_thread_ = thread;
        
        if (old_thread && old_thread != idle_thread_) {
            if (old_thread->get_state() == ThreadState::Running) {
                old_thread->set_state(ThreadState::Ready);
                add_thread(old_thread);
            }
            
            u64 current_time = arch::CPU::read_tsc();
            u64 cpu_time = current_time - last_schedule_time_;
            
            Process* old_process = old_thread->get_process();
            if (old_process) {
                old_process->update_statistics(cpu_time, 0);
            }
            
            statistics_.total_cpu_time += cpu_time;
        }
        
        thread->set_state(ThreadState::Running);
        
        u64 current_time = arch::CPU::read_tsc();
        last_schedule_time_ = current_time;
        statistics_.last_switch_time = current_time;
        statistics_.total_context_switches++;
        
        if (thread != idle_thread_) {
            statistics_.total_processes_scheduled++;
        } else {
            statistics_.idle_time += current_time - last_schedule_time_;
        }
        
        debug::log(debug::LogLevel::Trace, "SCHED",
                  "Context switch: %llu -> %llu",
                  old_thread ? old_thread->get_id() : 0,
                  thread->get_id());
    }

    void Scheduler::save_current_context() {
        if (current_thread_ && current_thread_ != idle_thread_) {
            RegisterState regs;
            asm volatile(
                "mov %%r15, %0\n"
                "mov %%r14, %1\n"
                "mov %%r13, %2\n"
                "mov %%r12, %3\n"
                "mov %%r11, %4\n"
                "mov %%r10, %5\n"
                "mov %%r9, %6\n"
                "mov %%r8, %7\n"
                "mov %%rdi, %8\n"
                "mov %%rsi, %9\n"
                "mov %%rbp, %10\n"
                "mov %%rbx, %11\n"
                "mov %%rdx, %12\n"
                "mov %%rcx, %13\n"
                "mov %%rax, %14\n"
                : "=m"(regs.r15), "=m"(regs.r14), "=m"(regs.r13),
                  "=m"(regs.r12), "=m"(regs.r11), "=m"(regs.r10),
                  "=m"(regs.r9), "=m"(regs.r8), "=m"(regs.rdi),
                  "=m"(regs.rsi), "=m"(regs.rbp), "=m"(regs.rbx),
                  "=m"(regs.rdx), "=m"(regs.rcx), "=m"(regs.rax)
            );
            
            asm volatile(
                "pushfq\n"
                "pop %0\n"
                : "=m"(regs.rflags)
            );
            
            asm volatile(
                "mov %%rsp, %0\n"
                : "=m"(regs.rsp)
            );
            
            current_thread_->save_context(&regs);
        }
    }

    void Scheduler::load_thread_context(Thread* thread) {
        if (thread && thread != idle_thread_) {
            RegisterState regs;
            thread->restore_context(&regs);
            
            mm::VirtualMemoryManager::instance()
                .switch_address_space(thread->get_process()->get_address_space());
            
            asm volatile(
                "mov %0, %%r15\n"
                "mov %1, %%r14\n"
                "mov %2, %%r13\n"
                "mov %3, %%r12\n"
                "mov %4, %%r11\n"
                "mov %5, %%r10\n"
                "mov %6, %%r9\n"
                "mov %7, %%r8\n"
                "mov %8, %%rdi\n"
                "mov %9, %%rsi\n"
                "mov %10, %%rbp\n"
                "mov %11, %%rbx\n"
                "mov %12, %%rdx\n"
                "mov %13, %%rcx\n"
                "mov %14, %%rax\n"
                : : "m"(regs.r15), "m"(regs.r14), "m"(regs.r13),
                  "m"(regs.r12), "m"(regs.r11), "m"(regs.r10),
                  "m"(regs.r9), "m"(regs.r8), "m"(regs.rdi),
                  "m"(regs.rsi), "m"(regs.rbp), "m"(regs.rbx),
                  "m"(regs.rdx), "m"(regs.rcx), "m"(regs.rax)
            );
            
            asm volatile(
                "push %0\n"
                "popfq\n"
                : : "m"(regs.rflags)
            );
            
            asm volatile(
                "mov %0, %%rsp\n"
                : : "m"(regs.rsp)
            );
        }
    }

    Process* Scheduler::get_current_process() {
        if (current_thread_ && current_thread_ != idle_thread_) {
            return current_thread_->get_process();
        }
        return nullptr;
    }

    void Scheduler::dump_run_queues() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "SCHED", "Run Queues:");
        for (usize i = 0; i < run_queues_.size(); i++) {
            const RunQueue& queue = run_queues_[i];
            debug::log(debug::LogLevel::Info, "SCHED",
                      "  Priority %u: %llu threads", i, queue.threads.size());
            for (const Thread* thread : queue.threads) {
                debug::log(debug::LogLevel::Info, "SCHED",
                          "    Thread %llu (PID: %llu, State: %u)",
                          thread->get_id(), thread->get_process()->get_pid(),
                          static_cast<u32>(thread->get_state()));
            }
        }
    }

    void Scheduler::dump_statistics() const {
        ScopedLock lock(lock_);
        
        debug::log(debug::LogLevel::Info, "SCHED", "Scheduler Statistics:");
        debug::log(debug::LogLevel::Info, "SCHED", "  Total Context Switches: %llu",
                  statistics_.total_context_switches);
        debug::log(debug::LogLevel::Info, "SCHED", "  Total Processes Scheduled: %llu",
                  statistics_.total_processes_scheduled);
        debug::log(debug::LogLevel::Info, "SCHED", "  Total CPU Time: %llu",
                  statistics_.total_cpu_time);
        debug::log(debug::LogLevel::Info, "SCHED", "  Idle Time: %llu",
                  statistics_.idle_time);
        debug::log(debug::LogLevel::Info, "SCHED", "  Last Switch Time: %llu",
                  statistics_.last_switch_time);
        
        double utilization = 0.0;
        if (statistics_.total_cpu_time + statistics_.idle_time > 0) {
            utilization = 100.0 * statistics_.total_cpu_time /
                         (statistics_.total_cpu_time + statistics_.idle_time);
        }
        debug::log(debug::LogLevel::Info, "SCHED", "  CPU Utilization: %.2f%%", utilization);
    }

    void Scheduler::on_timer_tick() {
        handle_timer_tick();
    }

    Scheduler& Scheduler::instance() {
        static Scheduler instance;
        return instance;
    }
}
