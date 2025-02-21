#pragma once

#include <cmath>

#include <eigen3/Eigen/Core>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat3<D1>
    rpy_to_mat3f(float r, float p, float y, Eigen::MatrixBase<D1>& m)
    {
        const float sgamma = std::sin(r);
        const float cgamma = std::cos(r);
        const float sbeta = std::sin(p);
        const float cbeta = std::cos(p);
        const float salpha = std::sin(y);
        const float calpha = std::cos(y);

        m(0, 0) = calpha * cbeta;
        m(0, 1) = calpha * sbeta * sgamma - salpha * cgamma;
        m(0, 2) = calpha * sbeta * cgamma + salpha * sgamma;

        m(1, 0) = salpha * cbeta;
        m(1, 1) = salpha * sbeta * sgamma + calpha * cgamma;
        m(1, 2) = salpha * sbeta * cgamma - calpha * sgamma;

        m(2, 0) = -sbeta;
        m(2, 1) = cbeta * sgamma;
        m(2, 2) = cbeta * cgamma;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat3<D1, D2>
    rpy_to_mat3f(const Eigen::MatrixBase<D1>& v, Eigen::MatrixBase<D2>& m)
    {
        rpy_to_mat3f(v(0), v(1), v(2), m);
    }

    inline Eigen::Matrix3f
    rpy_to_mat3f(float r, float p, float y)
    {
        Eigen::Matrix3f m;
        rpy_to_mat3f(r, p, y, m);
        return m;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix<typename D1::Scalar, 3, 3>>
    rpy_to_mat3f(const Eigen::MatrixBase<D1>& v)
    {
        Eigen::Matrix<typename D1::Scalar, 3, 3> m;
        rpy_to_mat3f(v, m);
        return m;
    }
} // namespace simox::math
