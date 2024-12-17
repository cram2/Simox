#pragma once

#include "mat4f_to_rpy.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1, Eigen::Vector3f>
    mat4f_to_xyyaw(const Eigen::MatrixBase<D1>& m)
    {
        return {m(0, 3), m(1, 3), mat4f_to_rpy(m)(2)};
    }

    template <class D1, class D2>
    inline meta::enable_if_mat4_vec3<D1, D2>
    mat4f_to_xyyaw(const Eigen::MatrixBase<D1>& m, Eigen::MatrixBase<D2>& xyyaw)
    {
        xyyaw = mat4f_to_xyyaw(m);
    }
} // namespace simox::math
