#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1, Eigen::AngleAxisf>
    mat4f_to_aa(const Eigen::MatrixBase<D1>& m)
    {
        return Eigen::AngleAxisf{m.template topLeftCorner<3, 3>()};
    }

    template <class D1>
    inline meta::enable_if_mat4<D1>
    mat4f_to_aa(const Eigen::MatrixBase<D1>& m, Eigen::AngleAxisf& aa)
    {
        aa = mat4f_to_aa(m);
    }
} // namespace simox::math
