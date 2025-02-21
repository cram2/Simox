#pragma once

#include <cmath>

#include <eigen3/Eigen/Core>

namespace simox::math
{
    using std::isfinite;

    template <class Derived>
    bool
    isfinite(const Eigen::MatrixBase<Derived>& mat)
    {
        for (int x = 0; x < mat.rows(); ++x)
        {
            for (int y = 0; y < mat.cols(); ++y)
            {
                if (!std::isfinite(mat(x, y)))
                {
                    return false;
                }
            }
        }
        return true;
    }
} // namespace simox::math
