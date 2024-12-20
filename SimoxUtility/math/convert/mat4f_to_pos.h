#pragma once

#include <Eigen/Dense>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat4_vec3<D1, D2>
    mat4f_to_pos(const Eigen::MatrixBase<D1>& m4, Eigen::MatrixBase<D2>& v)
    {
        v = m4.template topRightCorner<3, 1>();
    }

    template <class D1>
    inline meta::enable_if_mat4<D1, Eigen::Vector3f>
    mat4f_to_pos(const Eigen::MatrixBase<D1>& m4)
    {
        return m4.template topRightCorner<3, 1>();
    }
} // namespace simox::math
