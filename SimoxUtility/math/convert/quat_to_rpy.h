#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "mat3f_to_rpy.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_vec3<D1>
    quat_to_rpy(const Eigen::Quaternionf& q, Eigen::MatrixBase<D1>& v)
    {
        mat3f_to_rpy(q.toRotationMatrix(), v);
    }

    inline Eigen::Vector3f
    quat_to_rpy(const Eigen::Quaternionf& q)
    {
        return mat3f_to_rpy(q.toRotationMatrix());
    }
} // namespace simox::math
