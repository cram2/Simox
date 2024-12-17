#pragma once

#include <Eigen/Dense>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat3_mat4<D1, D2>
    pos_mat3f_to_mat4f(float x,
                       float y,
                       float z,
                       const Eigen::MatrixBase<D1>& m3,
                       Eigen::MatrixBase<D2>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = m3;
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2, class D3>
    inline meta::enable_if_vec3_mat3_mat4<D1, D2, D3>
    pos_mat3f_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                       const Eigen::MatrixBase<D2>& m3,
                       Eigen::MatrixBase<D3>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = m3;
        m4.template topRightCorner<3, 1>() = pos;
    }

    template <class D1>
    inline meta::enable_if_mat3<D1, Eigen::Matrix4f>
    pos_mat3f_to_mat4f(float x, float y, float z, const Eigen::MatrixBase<D1>& m3)
    {
        Eigen::Matrix4f m4;
        pos_mat3f_to_mat4f(x, y, z, m3, m4);
        return m4;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat3<D1, D2, Eigen::Matrix4f>
    pos_mat3f_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::MatrixBase<D2>& m3)
    {
        Eigen::Matrix4f m4;
        pos_mat3f_to_mat4f(pos, m3, m4);
        return m4;
    }
} // namespace simox::math
