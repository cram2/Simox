#pragma once


// STD/STL
#include <iterator>

// Simox
#include <SimoxUtility/math/sum.h>


namespace simox::math
{

    template <
        typename iter_t,
        typename element_t = typename std::iterator_traits<iter_t>::value_type>
    element_t
    mean(iter_t first, iter_t last)
    {
        return math::sum(first, last) * (1. / std::distance(first, last));
    }


    template <
        typename container_t,
        typename element_t =
            typename std::iterator_traits<
                decltype(std::begin(std::declval<container_t>()))>::value_type>
    element_t
    mean(const container_t& container)
    {
        return math::mean(std::begin(container), std::end(container));
    }

}
