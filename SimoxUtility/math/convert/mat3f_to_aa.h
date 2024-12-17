#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat3<D1, Eigen::AngleAxisf>
    mat3f_to_aa(const Eigen::MatrixBase<D1>& m)
    {
        return Eigen::AngleAxisf{m};
    }

    template <class D1>
    inline meta::enable_if_mat3<D1>
    mat3f_to_aa(const Eigen::MatrixBase<D1>& m, Eigen::AngleAxisf& aa)
    {
        aa = mat3f_to_aa(m);
    }
} // namespace simox::math
