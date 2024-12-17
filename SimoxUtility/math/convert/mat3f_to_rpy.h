#pragma once

#include <cmath>

#include <Eigen/Core>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat3_vec3<D1, D2>
    mat3f_to_rpy(const Eigen::MatrixBase<D1>& m, Eigen::MatrixBase<D2>& rpy)
    {
        // Eigen::Matrix4f A = mat.transpose();
        float alpha, beta, gamma;
        beta = std::atan2(-m(2, 0), std::sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0)));

        if (std::abs(beta - M_PI_2) < 1e-10)
        {
            alpha = 0;
            gamma = std::atan2(m(0, 1), m(1, 1));
        }
        else if (std::abs(beta + M_PI_2 * 0.5f) < 1e-10)
        {
            alpha = 0;
            gamma = -std::atan2(m(0, 1), m(1, 1));
        }
        else
        {
            float cb = 1.0 / std::cos(beta);
            alpha = std::atan2(m(1, 0) * cb, m(0, 0) * cb);
            gamma = std::atan2(m(2, 1) * cb, m(2, 2) * cb);
        }

        rpy(0) = gamma;
        rpy(1) = beta;
        rpy(2) = alpha;
    }

    template <class D1>
    inline meta::enable_if_mat3<D1, Eigen::Matrix<typename D1::Scalar, 3, 1>>
    mat3f_to_rpy(const Eigen::MatrixBase<D1>& m)
    {
        Eigen::Matrix<typename D1::Scalar, 3, 1> rpy;
        mat3f_to_rpy(m, rpy);
        return rpy;
    }
} // namespace simox::math
