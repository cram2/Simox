#pragma once

#include <Eigen/Dense>

#include "../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, Eigen::Vector3f>
    project_to_plane(const Eigen::MatrixBase<D1>& vec, const Eigen::MatrixBase<D2>& normal)
    {
        return vec - normal * (normal.dot(vec));
    }
} // namespace simox::math
