#pragma once


// STD/STL
#include <iterator>
#include <numeric>

// Simox
#include <SimoxUtility/math/zero.h>


namespace simox::math
{

    template <
        typename iter_t,
        typename element_t = typename std::iterator_traits<iter_t>::value_type>
    element_t
    sum(iter_t first, iter_t last, element_t zero)
    {
        return std::accumulate(first, last, zero);
    }


    template <
        typename iter_t,
        typename element_t = typename std::iterator_traits<iter_t>::value_type>
    element_t
    sum(iter_t first, iter_t last)
    {
        return math::sum(first, last, math::zero<element_t>());
    }


    template <
        typename container_t,
        typename element_t =
            typename std::iterator_traits<
                decltype(std::begin(std::declval<container_t>()))>::value_type>
    element_t
    sum(const container_t& container, element_t zero)
    {
        return math::sum(std::begin(container), std::end(container), zero);
    }


    template <
        typename container_t,
        typename element_t =
            typename std::iterator_traits<
                decltype(std::begin(std::declval<container_t>()))>::value_type>
    element_t
    sum(const container_t& container)
    {
        return math::sum(std::begin(container), std::end(container), math::zero<element_t>());
    }

}
