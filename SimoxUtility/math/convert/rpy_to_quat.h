#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "rpy_to_mat3f.h"

namespace simox::math
{
    inline Eigen::Quaternionf
    rpy_to_quat(float r, float p, float y)
    {
        return Eigen::Quaternionf{rpy_to_mat3f(r, p, y)};
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Quaternionf>
    rpy_to_quat(const Eigen::MatrixBase<D1>& v)
    {
        return Eigen::Quaternionf{rpy_to_mat3f(v)};
    }

    inline void
    rpy_to_quat(float r, float p, float y, Eigen::Quaternionf& q)
    {
        q = rpy_to_quat(r, p, y);
    }

    template <class D1>
    inline meta::enable_if_vec3<D1>
    rpy_to_quat(const Eigen::MatrixBase<D1>& v, Eigen::Quaternionf& q)
    {
        q = rpy_to_quat(v);
    }
} // namespace simox::math
