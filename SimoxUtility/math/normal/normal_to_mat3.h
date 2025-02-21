#pragma once


#include <eigen3/Eigen/Core>

namespace simox::math
{
    enum class CoordinateSystemAxis
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    template <class Scalar>
    Eigen::Matrix<Scalar, 3, 3>
    normal_to_mat3(Eigen::Matrix<Scalar, 3, 1> vector,
                   CoordinateSystemAxis axis = CoordinateSystemAxis::Z)
    {
        const Scalar len = vector.norm();
        if (len <= 1e-9)
        {
            throw std::invalid_argument{"the vector has no length!"};
        }
        vector /= len;

        const Eigen::Matrix<Scalar, 3, 1> vhelp = (vector(0) < 0.5f)
                                                      ? Eigen::Matrix<Scalar, 3, 1>::UnitX()
                                                      : Eigen::Matrix<Scalar, 3, 1>::UnitY();

        const Eigen::Matrix<Scalar, 3, 1> v2 = vector.cross(vhelp).normalized();
        const Eigen::Matrix<Scalar, 3, 1> v3 = vector.cross(v2).normalized();

        Eigen::Matrix<Scalar, 3, 3> m;
        m.template col(static_cast<int>(axis)) = vector;
        m.template col((static_cast<int>(axis) + 1) % 3) = v2;
        m.template col((static_cast<int>(axis) + 2) % 3) = v3;

        return m;
    }
} // namespace simox::math
