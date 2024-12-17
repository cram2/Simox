#pragma once

#include <Eigen/Dense>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1>
    pos_to_mat4f(float x, float y, float z, Eigen::MatrixBase<D1>& m4)
    {
        m4.setIdentity();
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_to_mat4f(const Eigen::MatrixBase<D1>& pos, Eigen::MatrixBase<D2>& m4)
    {
        m4.setIdentity();
        m4.template topRightCorner<3, 1>() = pos;
    }

    inline Eigen::Matrix4f
    pos_to_mat4f(float x, float y, float z)
    {
        Eigen::Matrix4f m4;
        pos_to_mat4f(x, y, z, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_to_mat4f(const Eigen::MatrixBase<D1>& pos)
    {
        Eigen::Matrix4f m4;
        pos_to_mat4f(pos, m4);
        return m4;
    }
} // namespace simox::math
