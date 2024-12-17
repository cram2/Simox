#include "CountingSemaphore.h"

namespace simox::threads
{

    CountingSemaphore::CountingSemaphore()
    {
    }

    CountingSemaphore::CountingSemaphore(unsigned int count) : _count(count)
    {
    }

    CountingSemaphore::CountingSemaphore(unsigned int count, unsigned int maxCount) :
        _count(count), _maxCount(maxCount)
    {
    }

    void
    CountingSemaphore::notify()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_maxCount || _count < *_maxCount)
        {
            ++_count;
        }
        _condition.notify_one();
    }

    void
    CountingSemaphore::wait()
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _condition.wait(lock, [this]() { return _count > 0; });
        --_count;
    }

    bool
    CountingSemaphore::try_wait()
    {
        std::unique_lock<std::mutex> lock(_mutex);
        if (_count > 0)
        {
            --_count;
            return true;
        }
        return false;
    }

} // namespace simox::threads
