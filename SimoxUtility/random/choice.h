#pragma once

#include <random>
#include <stdexcept>
#include <vector>

#include <SimoxUtility/error/SimoxError.h>


namespace simox::random
{
    /**
     * Return a random element from the non-empty vector items.
     *
     * @throw `std::out_of_range` if items is empty.
     */
    template <class T, class RandomEngineT>
    const T&
    choice(const std::vector<T>& items, RandomEngineT& gen)
    {
        if (items.empty())
        {
            throw error::SimoxError("Cannot choose an item from an empty vector.");
        }

        std::uniform_int_distribution<size_t> distrib(0, items.size() - 1);
        std::size_t index = distrib(gen);
        return items.at(index);
    }

    template <class T>
    const T&
    choice(const std::vector<T>& items)
    {
        std::default_random_engine gen{std::random_device{}()};
        return choice(items, gen);
    }

} // namespace simox::random
