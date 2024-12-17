#pragma once

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "rpy_to_mat3f.h"

namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1>
    rpy_to_mat4f(float r, float p, float y, Eigen::MatrixBase<D1>& m4)
    {
        m4.setIdentity();
        m4.template topLeftCorner<3, 3>() = rpy_to_mat3f(r, p, y);
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    rpy_to_mat4f(const Eigen::MatrixBase<D1>& v, Eigen::MatrixBase<D2>& m4)
    {
        rpy_to_mat4f(v(0), v(1), v(2), m4);
    }

    inline Eigen::Matrix4f
    rpy_to_mat4f(float r, float p, float y)
    {
        Eigen::Matrix4f m4;
        rpy_to_mat4f(r, p, y, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix<typename D1::Scalar, 4, 4>>
    rpy_to_mat4f(const Eigen::MatrixBase<D1>& v)
    {
        Eigen::Matrix<typename D1::Scalar, 4, 4> m4;
        rpy_to_mat4f(v, m4);
        return m4;
    }
} // namespace simox::math
