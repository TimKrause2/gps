#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <class T>
class ThreadQueue
{
private:
    std::queue<T> queue;
    mutable std::mutex mutex;
    std::condition_variable condition;
    bool stop_thread = false; // Flag to signal the consumer to stop

public:
    void push(T data) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(std::move(data));
        condition.notify_one(); // Notify one waiting thread that data is ready
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] { return !queue.empty() || stop_thread; }); // Wait if queue is empty

        if (stop_thread && queue.empty()) {
            return nullptr; // Return null to signal thread termination
        }

        T data = std::move(queue.front());
        queue.pop();
        return data;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex);
        stop_thread = true;
        condition.notify_all(); // Notify all waiting threads to check the stop flag
    }

    bool empty(){
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    }

    int size(){
        std::lock_guard<std::mutex> lock(mutex);
        return queue.size();
    }
};
