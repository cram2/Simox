#pragma once

#include <Eigen/Core>

#include <SimoxUtility/error/SimoxError.h>

namespace simox::math
{
    bool is_rotation_matrix(const Eigen::Matrix3f& rotation, float precision = 1e-6f);

    /**
     * @brief Checks whether `rotation` is a valid rotation matrix.
     *
     * Check whether `rotation` is orthogonal and has determinant 1.0.
     *
     * @param rotation The rotation matrix to check.
     * @param precision The precision of floating point comparison.
     * @throw `simox::error::InvalidRotationMatrix` If any condition is not fulfilled.
     */
    void check_rotation_matrix(const Eigen::Matrix3f& rotation, float precision = 1e-6f);

} // namespace simox::math

namespace simox::error
{
    /// Indicates that a rotation matrix was invalid.
    class InvalidRotationMatrix : public SimoxError
    {
    public:
        InvalidRotationMatrix(Eigen::Matrix3f matrix, const std::string& message = "");

        static std::string make_msg(Eigen::Matrix3f matrix, const std::string& message = "");
    };
} // namespace simox::error
