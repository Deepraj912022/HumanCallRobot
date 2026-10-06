#ifndef HUMAN_CALL_ROBOT_CORE_MUTEX_HPP_
#define HUMAN_CALL_ROBOT_CORE_MUTEX_HPP_

#include <atomic>

namespace human_call_robot {

class SpinMutex {
public:
    SpinMutex() = default;
    
    void lock() {
        while (flag_.test_and_set(std::memory_order_acquire)) {
            // spin-wait
        }
    }

    void unlock() {
        flag_.clear(std::memory_order_release);
    }

private:
    std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

class SpinLockGuard {
public:
    explicit SpinLockGuard(SpinMutex& mutex) : mutex_(mutex) {
        mutex_.lock();
    }

    ~SpinLockGuard() {
        mutex_.unlock();
    }

private:
    SpinMutex& mutex_;
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_MUTEX_HPP_
