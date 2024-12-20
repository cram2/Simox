#pragma once


// STD/STL
#include <type_traits>

// Simox
#include <SimoxUtility/meta/eigen/is_eigen_matrix.h>

namespace simox::math
{

    template <typename t>
    t
    zero()
    {
        if constexpr (std::is_arithmetic_v<t>)
        {
            return 0;
        }
        else if constexpr (simox::meta::is_eigen_matrix_v<t>)
        {
            return t::Zero();
        }
        else
        {
            static_assert(not std::is_same_v<t, t>, "Cannot deduce zero value.");
        }
    }

} // namespace simox::math
