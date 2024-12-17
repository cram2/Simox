#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1>
    quat_to_mat4f(const Eigen::Quaternionf& q, Eigen::MatrixBase<D1>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = q.toRotationMatrix();
    }

    inline Eigen::Matrix4f
    quat_to_mat4f(const Eigen::Quaternionf& q)
    {
        Eigen::Matrix4f m4;
        quat_to_mat4f(q, m4);
        return m4;
    }
} // namespace simox::math
