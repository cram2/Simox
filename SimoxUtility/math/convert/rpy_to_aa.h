#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "rpy_to_mat3f.h"

namespace simox::math
{
    inline Eigen::AngleAxisf
    rpy_to_aa(float r, float p, float y)
    {
        return Eigen::AngleAxisf{rpy_to_mat3f(r, p, y)};
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::AngleAxisf>
    rpy_to_aa(const Eigen::MatrixBase<D1>& v)
    {
        return Eigen::AngleAxisf{rpy_to_mat3f(v)};
    }

    inline void
    rpy_to_aa(float r, float p, float y, Eigen::AngleAxisf& aa)
    {
        aa = rpy_to_aa(r, p, y);
    }

    template <class D1>
    inline meta::enable_if_vec3<D1>
    rpy_to_aa(const Eigen::MatrixBase<D1>& v, Eigen::AngleAxisf& aa)
    {
        aa = rpy_to_aa(v);
    }
} // namespace simox::math
