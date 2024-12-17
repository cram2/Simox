#pragma once

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "rpy_to_mat4f.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1>
    xyyaw_to_mat4f(float x, float y, float yaw, Eigen::MatrixBase<D1>& m)
    {
        m = rpy_to_mat4f(0, 0, yaw);
        m(0, 3) = x;
        m(1, 3) = y;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    xyyaw_to_mat4f(const Eigen::MatrixBase<D1>& v, Eigen::MatrixBase<D2>& m)
    {
        xyyaw_to_mat4f(v(0), v(1), v(2), m);
    }

    inline Eigen::Matrix4f
    xyyaw_to_mat4f(float x, float y, float yaw)
    {
        Eigen::Matrix4f m;
        xyyaw_to_mat4f(x, y, yaw, m);
        return m;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    xyyaw_to_mat4f(const Eigen::MatrixBase<D1>& v)
    {
        Eigen::Matrix4f m;
        xyyaw_to_mat4f(v, m);
        return m;
    }
} // namespace simox::math
