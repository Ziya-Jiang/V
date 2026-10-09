#ifndef THREAD_SAFE_COMMAND_H
#define THREAD_SAFE_COMMAND_H

#include <functional>
#include <mutex>

template <typename T>
class ThreadSafeCommand{
    public:
        void updateCommand(std::function<void(T&)> edit_func) {
            std::lock_guard<std::mutex> lock(mutex_);
            return edit_func(command_);
        }

        void viewCommand(std::function<void(const T&)> read_func) const{
            std::lock_guard<std::mutex> lock(mutex_);
            return read_func(command_);
        }

    private:
        T command_;                     // The shared resource
        mutable std::mutex mutex_;      // The lock guarding the resource
};

#endif