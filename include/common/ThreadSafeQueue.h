#ifndef THREADSAFE_QUEUE_H
#define THREADSAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

namespace SensorFusion
{

    template <typename T>
    class ThreadSafeQueue
    {
    public:
        ThreadSafeQueue() : shutdown_(false) {}

        void push(const T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(item);
            cv_.notify_one();
        }

        void push(T &&item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(item));
            cv_.notify_one();
        }

        std::optional<T> pop()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]
                     { return !queue_.empty() || shutdown_; });

            if (shutdown_ && queue_.empty())
            {
                return std::nullopt;
            }

            T item = std::move(queue_.front());
            queue_.pop();
            return item;
        }

        std::optional<T> tryPop()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty())
            {
                return std::nullopt;
            }
            T item = std::move(queue_.front());
            queue_.pop();
            return item;
        }

        bool empty() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return queue_.empty();
        }

        size_t size() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return queue_.size();
        }

        void shutdown()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
            cv_.notify_all();
        }

    private:
        std::queue<T> queue_;
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        bool shutdown_;
    };

} // namespace SensorFusion

#endif // THREADSAFE_QUEUE_H
