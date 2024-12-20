#pragma once

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "mat3f_to_rpy.h"

namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat4_vec3<D1, D2>
    mat4f_to_rpy(const Eigen::MatrixBase<D1>& m, Eigen::MatrixBase<D2>& rpy)
    {
        mat3f_to_rpy(m.template topLeftCorner<3, 3>(), rpy);
    }

    template <class D1>
    inline meta::enable_if_mat4<D1, Eigen::Matrix<typename D1::Scalar, 3, 1>>
    mat4f_to_rpy(const Eigen::MatrixBase<D1>& m)
    {
        return mat3f_to_rpy(m.template topLeftCorner<3, 3>());
    }
} // namespace simox::math
