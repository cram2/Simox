#pragma once

#include <eigen3/Eigen/Core>

#include <SimoxUtility/meta/eigen/enable_if_compile_time_size.h>

namespace simox::math
{
    template <class D1, class D2>
    meta::enable_if_mat4_vec3<D1, D2, Eigen::Matrix<typename Eigen::MatrixBase<D1>::Scalar, 3, 1>>
    transform_direction(const Eigen::MatrixBase<D1>& m4, const Eigen::MatrixBase<D2>& v)
    {
        return m4.template topLeftCorner<3, 3>() * v;
    }

    template <class D1, class D2, class Scalar = typename Eigen::MatrixBase<D1>::Scalar>
    meta::enable_if_mat4_vec3<D1, D2, Eigen::Matrix<Scalar, 3, 1>>
    transform_position(const Eigen::MatrixBase<D1>& m4, const Eigen::MatrixBase<D2>& v)
    {
        return m4.template topLeftCorner<3, 3>() * v + m4.template topRightCorner<3, 1>();
    }

    template <class D1, class Scalar = typename Eigen::MatrixBase<D1>::Scalar>
    meta::enable_if_mat4<D1, Eigen::Matrix<Scalar, 3, 1>>
    transform_position(const Eigen::MatrixBase<D1>& m4, Scalar x, Scalar y, Scalar z)
    {
        return transform_position(m4, Eigen::Matrix<Scalar, 3, 1>{x, y, z});
    }
} // namespace simox::math
