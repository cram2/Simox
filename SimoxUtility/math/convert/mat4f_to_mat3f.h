#pragma once

#include <Eigen/Dense>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat4_mat3<D1, D2>
    mat4f_to_mat3f(const Eigen::MatrixBase<D1>& m4, Eigen::MatrixBase<D2>& m3)
    {
        m3 = m4.template topLeftCorner<3, 3>();
    }

    template <class D1>
    inline meta::enable_if_mat4<D1, Eigen::Matrix3f>
    mat4f_to_mat3f(const Eigen::MatrixBase<D1>& m4)
    {
        return m4.template topLeftCorner<3, 3>();
    }
} // namespace simox::math
