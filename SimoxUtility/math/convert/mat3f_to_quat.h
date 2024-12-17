#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat3<D1, Eigen::Quaternionf>
    mat3f_to_quat(const Eigen::MatrixBase<D1>& m)
    {
        return Eigen::Quaternionf{m};
    }

    template <class D1>
    inline meta::enable_if_mat3<D1>
    mat3f_to_quat(const Eigen::MatrixBase<D1>& m, Eigen::Quaternionf& q)
    {
        q = mat3f_to_quat(m);
    }
} // namespace simox::math
