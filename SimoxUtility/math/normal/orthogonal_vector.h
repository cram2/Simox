#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace simox::math
{
    template <class Scalar>
    Eigen::Matrix<Scalar, 3, 1>
    orthogonal_vector(Eigen::Matrix<Scalar, 3, 1> vector)
    {
        const Scalar len = vector.norm();
        if (len <= 1e-9)
        {
            throw std::invalid_argument{"the vector has no length!"};
        }
        vector /= len;

        const Eigen::Matrix<Scalar, 3, 1> v2 = (std::abs(vector(0)) < 0.5f)
                                                   ? Eigen::Matrix<Scalar, 3, 1>::UnitX()
                                                   : Eigen::Matrix<Scalar, 3, 1>::UnitY();
        return vector.cross(v2).normalized();
    }
} // namespace simox::math
