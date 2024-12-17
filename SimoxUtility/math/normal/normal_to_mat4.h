#pragma once

#include "normal_to_mat3.h"

namespace simox::math
{
    template <class Scalar>
    Eigen::Matrix<Scalar, 4, 4>
    normal_to_mat4(Eigen::Matrix<Scalar, 3, 1> vector,
                   CoordinateSystemAxis axis = CoordinateSystemAxis::Z)
    {
        Eigen::Matrix<Scalar, 4, 4> m = Eigen::Matrix<Scalar, 4, 4>::Identity();
        m.template topLeftCorner<3, 3>() = normal_to_mat3(vector, axis);
        return m;
    }

    template <class Scalar>
    Eigen::Matrix<Scalar, 4, 4>
    normal_pos_to_mat4(Eigen::Matrix<Scalar, 3, 1> vector,
                       Eigen::Matrix<Scalar, 3, 1> position,
                       CoordinateSystemAxis axis = CoordinateSystemAxis::Z)
    {
        Eigen::Matrix<Scalar, 4, 4> m = Eigen::Matrix<Scalar, 4, 4>::Identity();
        m.template topLeftCorner<3, 3>() = normal_to_mat3(vector, axis);
        m.template topRightCorner<3, 1>() = position;
        return m;
    }
} // namespace simox::math
