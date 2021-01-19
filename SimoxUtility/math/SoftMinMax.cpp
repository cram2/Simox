#include "SoftMinMax.h"

#include <cmath>
#include <stdexcept>
#include <sstream>


namespace simox::math
{

    SoftMinMax::SoftMinMax()
    {
        reset(0, 1);
    }

    SoftMinMax::SoftMinMax(float percentile, std::size_t numValues)
    {
        reset(percentile, numValues);
    }

    void SoftMinMax::reset(float percentile, std::size_t numValues)
    {
        minQueue = MinQueue();
        maxQueue = MaxQueue();

        if (percentile < 0 || percentile > 0.5f)
        {
            std::stringstream msg;
            msg << "percentile must be in [0, 0.5], but was " << percentile << ".";
            throw std::invalid_argument(msg.str());
        }
        if (numValues == 0)
        {
            std::stringstream msg;
            msg << "numValues must be > 0, but was " << numValues;
            throw std::invalid_argument(msg.str());
        }
        this->percentile = percentile;
        this->num_elements = numValues;

        allowed_heap_size_cache = allowedHeapSize();
    }

    void SoftMinMax::add(float value)
    {
        if (minQueue.size() != maxQueue.size())
        {
            std::stringstream msg;
            msg << "Invariant violated: minQueue.size() == maxQueue.size(), "
                << "but were " << minQueue.size() << " and " << maxQueue.size();
            throw std::logic_error(msg.str());
        }

        if (minQueue.size() < allowed_heap_size_cache)
        {
            // Heaps not full yet
            minQueue.push(value);
            maxQueue.push(value);
            return;
        }

        // Heaps are full

        // Check minQueue
        if (value < minQueue.top())
        {
            // insert and pop
            minQueue.push(value);
            minQueue.pop();
        } // else ignore value

        // Check maxQueue
        if (value > maxQueue.top())
        {
            maxQueue.push(value);
            maxQueue.pop();
        }

    }

    float SoftMinMax::getSoftMin() const
    {
        if (maxQueue.empty())
        {
            throw std::out_of_range("Calling getSoftMin() but no element was added.");
        }
        return minQueue.top();
    }

    float SoftMinMax::getSoftMax() const
    {
        if (maxQueue.empty())
        {
            throw std::out_of_range("Calling getSoftMin() but no element was added.");
        }

        return maxQueue.top();
    }

    std::size_t SoftMinMax::numOutsideSoftMinMax() const
    {
        return size_t(std::ceil(percentile * num_elements));
    }

    std::size_t SoftMinMax::allowedHeapSize() const
    {
        return std::max(std::size_t(1), numOutsideSoftMinMax() + 1);
    }

}
