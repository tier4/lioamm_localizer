#include <deque>
#include <mutex>
#include <optional>

template <typename T>
class ConcurrentQueue
{
public:
  ConcurrentQueue() {}
  ~ConcurrentQueue() {}

  T operator[](const std::size_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_[index];
  }

  T front() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::move(queue_.front());
  }

  T back() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::move(queue_.back());
  }

  void push_back(const T & data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(data));
  }

  std::optional<T> pop_front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }

    T data = std::move(queue_.front());
    queue_.pop_front();

    return data;
  }
  std::optional<T> pop_back()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }

    T data = std::move(queue_.back());
    queue_.pop_back();

    return data;
  }

  std::deque<T> pop_all()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::deque<T> data;
    while (!queue_.empty()) {
      data.push_back(std::move(queue_.front()));
      queue_.pop_front();
    }

    return data;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    return;
  }

private:
  std::deque<T> queue_;
  mutable std::mutex mutex_;
};
