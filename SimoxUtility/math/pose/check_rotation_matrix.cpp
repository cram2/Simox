#include "check_rotation_matrix.h"

#include <sstream>

#include <Eigen/Dense> // for .determinant()

#include "orthogonalize.h"

bool
simox::math::is_rotation_matrix(const Eigen::Matrix3f& rotation, float precision)
{
    if (!simox::math::is_matrix_orthogonal(rotation, precision))
    {
        return false;
    }
    if (std::abs(rotation.determinant() - 1.f) > precision)
    {
        return false;
    }
    return true;
}

void
simox::math::check_rotation_matrix(const Eigen::Matrix3f& rotation, float precision)
{
    // ARMARX_CHECK(simox::math::is_matrix_orthogonal(rotation)) << VAROUT(rotation);
    // ARMARX_CHECK_CLOSE(rotation.determinant(), 1.f, 1e-6f) << VAROUT(rotation);

    if (!simox::math::is_matrix_orthogonal(rotation, precision))
    {
        throw simox::error::InvalidRotationMatrix(rotation, "Rotation matrix is not orthogonal.");
    }
    if (std::abs(rotation.determinant() - 1.f) > precision)
    {
        throw simox::error::InvalidRotationMatrix(rotation,
                                                  "Determinant of rotation matrix is not 1.0.");
    }
}

namespace simox::error
{

    InvalidRotationMatrix::InvalidRotationMatrix(const Eigen::Matrix3f matrix,
                                                 const std::string& message) :
        SimoxError(make_msg(matrix, message))
    {
    }

    std::string
    InvalidRotationMatrix::make_msg(const Eigen::Matrix3f matrix, const std::string& message)
    {
        std::stringstream ss;
        if (!message.empty())
        {
            ss << message << "\n";
        }
        ss << "Matrix R: \n" << matrix << "\n";
        ss << "Determinant: " << matrix.determinant() << "\n";
        ss << "Matrix R * R^T: \n" << matrix * matrix.transpose() << "\n";
        return ss.str();
    }
} // namespace simox::error
