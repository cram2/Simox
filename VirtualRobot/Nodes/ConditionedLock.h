#pragma once

#include <memory>
#include <mutex>

template <class T>
class ConditionedLock
{
private:
    T _lock;
    bool _enabled;
public:
    ConditionedLock(std::recursive_mutex&   mutex, bool enabled) :
        _lock(mutex, std::defer_lock), _enabled(enabled)
    {
        if (_enabled)
        {
            _lock.lock();
        }
    }
    ~ConditionedLock()
    {
        if (_enabled)
        {
            _lock.unlock();
        }
    }
};

using ReadLock = ConditionedLock<std::unique_lock<std::recursive_mutex>>;
using WriteLock = ConditionedLock<std::unique_lock<std::recursive_mutex>>;

using ReadLockPtr = std::shared_ptr<ReadLock>;
using WriteLockPtr = std::shared_ptr<WriteLock>;
