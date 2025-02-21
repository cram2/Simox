#pragma once


// Eigen
#include <eigen3/Eigen/Core>

namespace simox::math
{

    template <int rows>
    Eigen::Matrix<float, rows, 1>
    norm_min(const Eigen::Matrix<float, rows, 1>& v, const float min)
    {
        if (v.norm() < min)
        {
            return v.normalized() * min;
        }

        return v;
    }

} // namespace simox::math
