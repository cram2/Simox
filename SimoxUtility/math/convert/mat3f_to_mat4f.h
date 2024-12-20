#pragma once

#include <Eigen/Dense>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat3_mat4<D1, D2>
    mat3f_to_mat4f(const Eigen::MatrixBase<D1>& m, Eigen::MatrixBase<D2>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = m;
    }

    template <class D1>
    inline meta::enable_if_mat3<D1, Eigen::Matrix4f>
    mat3f_to_mat4f(const Eigen::MatrixBase<D1>& m)
    {
        Eigen::Matrix4f m4;
        mat3f_to_mat4f(m, m4);
        return m4;
    }
} // namespace simox::math
