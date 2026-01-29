#ifndef NANOKOTON_SCHEDULER_HPP
#define NANOKOTON_SCHEDULER_HPP

#include <nanokoton/types.hpp>
#include <nanokoton/task/process.hpp>
#include <nanokoton/lib/vector.hpp>
#include <nanokoton/lib/queue.hpp>
#include <nanokoton/lib/mutex.hpp>
#include <nanokoton/lib/bitmap.hpp>

namespace nk::task {
    enum class SchedulingPolicy {
        RoundRobin,
        Priority,
        RealTime,
        Fair
    };

    struct SchedulingStatistics {
        u64 total_context_switches;
        u64 total_processes_scheduled;
        u64 total_cpu_time;
        u64 idle_time;
        u64 last_switch_time;
    };

    class Scheduler {
    private:
        struct RunQueue {
            Vector<Thread*> threads;
            usize current_index;
            u64 time_slice;
            u32 priority_level;
        };
        
        Vector<RunQueue> run_queues_;
        Thread* idle_thread_;
        Thread* current_thread_;
        Process* idle_process_;
        
        SchedulingPolicy policy_;
        u64 time_slice_default_;
        u64 last_schedule_time_;
        u64 timer_ticks_;
        
        SchedulingStatistics statistics_;
        Mutex lock_;
        
        Bitmap cpu_affinity_;
        u32 cpu_count_;
        
        void initialize_idle_thread();
        void switch_to_thread(Thread* thread);
        void save_current_context();
        void load_thread_context(Thread* thread);
        
        Thread* select_next_thread();
        void update_thread_priorities();
        void handle_timer_tick();
        
        bool validate_thread(Thread* thread);
        void cleanup_dead_threads();
        
        u64 calculate_time_slice(Thread* thread);
        u32 calculate_priority(Thread* thread);
        
    public:
        Scheduler();
        ~Scheduler();
        
        bool init();
        void start();
        
        void add_thread(Thread* thread);
        void remove_thread(Thread* thread);
        void yield();
        void sleep(u64 milliseconds);
        void wake_up(Thread* thread);
        
        Thread* get_current_thread() { return current_thread_; }
        Process* get_current_process();
        
        void set_scheduling_policy(SchedulingPolicy policy) { policy_ = policy; }
        SchedulingPolicy get_scheduling_policy() const { return policy_; }
        
        void set_time_slice(u64 time_slice) { time_slice_default_ = time_slice; }
        u64 get_time_slice() const { return time_slice_default_; }
        
        const SchedulingStatistics& get_statistics() const { return statistics_; }
        
        void dump_run_queues() const;
        void dump_statistics() const;
        
        void on_timer_tick();
        
        static Scheduler& instance();
    };
}
#endif
