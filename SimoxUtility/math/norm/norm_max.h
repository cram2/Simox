#pragma once


// Eigen
#include <Eigen/Core>

namespace simox::math
{

    template <int rows>
    Eigen::Matrix<float, rows, 1>
    norm_max(const Eigen::Matrix<float, rows, 1>& v, const float max)
    {
        if (v.norm() > max)
        {
            return v.normalized() * max;
        }

        return v;
    }

} // namespace simox::math
