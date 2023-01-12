#pragma once

#include <random>
#include <iterator>

#include <SimoxUtility/error/SimoxError.h>


namespace simox::random
{
    /**
     * Return a random element from the non-empty iterable items.
     *
     * @throw `std::out_of_range` if items is empty.
     */
    template <class IterableT, class RandomEngineT>
    const typename IterableT::value_type&
    choice(const IterableT& items, RandomEngineT& gen)
    {
        if (items.empty())
        {
            throw error::SimoxError("Cannot choose an item from an empty vector.");
        }

        std::uniform_int_distribution<size_t> distrib(0, items.size() - 1);
        std::size_t index = distrib(gen);
        auto it = items.begin();
        std::advance(it, index);
        return *it;
    }


    template <class IterableT>
    const typename IterableT::value_type&
    choice(const IterableT& items)
    {
        std::default_random_engine gen{std::random_device{}()};
        return choice(items, gen);
    }

} // namespace simox::random
