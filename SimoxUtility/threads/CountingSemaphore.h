#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>

namespace simox::threads
{

    /**
     * @brief A counting semaphore.
     *
     * Threads can enter when the internal count is > 0.
     * Notifiying the semaphore increments the count and allows threads to enter.
     * A thread can wait until it may enter. When it enters, it decrements
     * the internal count.
     * An optional max count can limit the value of count (useful e.g. when your
     * buffer has a limited number of items).
     *
     * A counting semaphore can be used e.g. in Producer-Consumer patterns.
     * The producer signals new jobs/items via `notify()`, while the consumer waits
     * for new jobs via `wait()`.
     */
    class CountingSemaphore
    {
    public:
        /// Construct an initially blocking semaphore (initial count 0) without max count.
        CountingSemaphore();
        /**
         * @brief Construct a semaphore with the given initial count without max count.
         * @param count The initial count (0 to block initially).
         */
        CountingSemaphore(unsigned int count);
        /**
         * @brief Construct a semaphore with the given initial count and max count.
         * @param count The initial count (0 to block initially).
         * @param maxCount An optional count limit (1 for a binary semaphore).
         */
        CountingSemaphore(unsigned int count, unsigned int maxCount);

        /**
         * @brief Signal that one waiting thread may continue.
         *
         * Also known as `post()` or `signal()`.
         * Increments the count by 1, if it is below the optional max count.
         */
        void notify();

        /**
         * @brief Wait until a thread may enter.
         *
         * Decrements the count when resuming.
         */
        void wait();

        /**
         * @brief Try to enter.
         *
         * If the semaphore is currently blocking, return false.
         * If the semaphore is free (count > 0), decrement the count and return true.
         *
         * @return True if entering was successful, false if semaphore was blocking.
         */
        bool try_wait();


    private:
        /// The mutex for _condition and _count.
        std::mutex _mutex;
        /// The condition variable to wake up waking threads.
        std::condition_variable _condition;

        /// The current count. Waiting threads may enter when > 0.
        unsigned int _count = 0;

        /// An optional maximal count. All `notifiy()`s increasing the counter above maxCount are ignored.
        std::optional<unsigned int> _maxCount = std::nullopt;
    };

} // namespace simox::threads
