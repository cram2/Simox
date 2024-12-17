#pragma once

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "rpy_to_mat4f.h"

//out param version
namespace simox::math
{

    template <class D1>
    inline meta::enable_if_mat4<D1>
    pos_rpy_to_mat4f(float x,
                     float y,
                     float z,
                     float roll,
                     float pitch,
                     float yaw,
                     Eigen::MatrixBase<D1>& m4)
    {
        rpy_to_mat4f(roll, pitch, yaw, m4);
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_rpy_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                     float roll,
                     float pitch,
                     float yaw,
                     Eigen::MatrixBase<D2>& m4)
    {
        rpy_to_mat4f(roll, pitch, yaw, m4);
        m4.template topRightCorner<3, 1>() = pos;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_rpy_to_mat4f(float x,
                     float y,
                     float z,
                     const Eigen::MatrixBase<D1>& rpy,
                     Eigen::MatrixBase<D2>& m4)
    {
        rpy_to_mat4f(rpy, m4);
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2, class D3>
    inline meta::enable_if_vec3_vec3_mat4<D1, D2, D3>
    pos_rpy_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                     const Eigen::MatrixBase<D2>& rpy,
                     Eigen::MatrixBase<D3>& m4)
    {
        rpy_to_mat4f(rpy, m4);
        m4.template topRightCorner<3, 1>() = pos;
    }
} // namespace simox::math

//return version
namespace simox::math
{
    inline Eigen::Matrix4f
    pos_rpy_to_mat4f(float x, float y, float z, float roll, float pitch, float yaw)
    {
        Eigen::Matrix4f m4;
        pos_rpy_to_mat4f(x, y, z, roll, pitch, yaw, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_rpy_to_mat4f(const Eigen::MatrixBase<D1>& pos, float roll, float pitch, float yaw)
    {
        Eigen::Matrix4f m4;
        pos_rpy_to_mat4f(pos, roll, pitch, yaw, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_rpy_to_mat4f(float x, float y, float z, const Eigen::MatrixBase<D1>& rpy)
    {
        Eigen::Matrix4f m4;
        pos_rpy_to_mat4f(x, y, z, rpy, m4);
        return m4;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, Eigen::Matrix4f>
    pos_rpy_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::MatrixBase<D2>& rpy)
    {
        Eigen::Matrix4f m4;
        pos_rpy_to_mat4f(pos, rpy, m4);
        return m4;
    }
} // namespace simox::math
