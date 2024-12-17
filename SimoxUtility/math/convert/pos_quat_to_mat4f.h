#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{

    template <class D1>
    inline meta::enable_if_mat4<D1>
    pos_quat_to_mat4f(float x,
                      float y,
                      float z,
                      const Eigen::Quaternionf& q,
                      Eigen::MatrixBase<D1>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = q.toRotationMatrix();
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_quat_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                      const Eigen::Quaternionf& q,
                      Eigen::MatrixBase<D2>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = q.toRotationMatrix();
        m4.template topRightCorner<3, 1>() = pos;
    }

    inline Eigen::Matrix4f
    pos_quat_to_mat4f(float x, float y, float z, const Eigen::Quaternionf& q)
    {
        Eigen::Matrix4f m4;
        pos_quat_to_mat4f(x, y, z, q, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_quat_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::Quaternionf& q)
    {
        Eigen::Matrix4f m4;
        pos_quat_to_mat4f(pos, q, m4);
        return m4;
    }

    // Legacy overloads of pos_mat3f_to_mat4f().

    template <class D1>
    inline meta::enable_if_mat4<D1>
    pos_mat3f_to_mat4f(float x,
                       float y,
                       float z,
                       const Eigen::Quaternionf& q,
                       Eigen::MatrixBase<D1>& m4)
    {
        pos_quat_to_mat4f(x, y, z, q, m4);
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_mat3f_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                       const Eigen::Quaternionf& q,
                       Eigen::MatrixBase<D2>& m4)
    {
        pos_quat_to_mat4f(pos, q, m4);
    }

    inline Eigen::Matrix4f
    pos_mat3f_to_mat4f(float x, float y, float z, const Eigen::Quaternionf& q)
    {
        return pos_quat_to_mat4f(x, y, z, q);
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_mat3f_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::Quaternionf& q)
    {
        return pos_quat_to_mat4f(pos, q);
    }
} // namespace simox::math
