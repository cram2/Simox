#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat3<D1>
    quat_to_mat3f(const Eigen::Quaternionf& q, Eigen::MatrixBase<D1>& m3)
    {
        m3 = q.toRotationMatrix();
    }

    inline Eigen::Matrix3f
    quat_to_mat3f(const Eigen::Quaternionf& q)
    {
        return q.toRotationMatrix();
    }
} // namespace simox::math
