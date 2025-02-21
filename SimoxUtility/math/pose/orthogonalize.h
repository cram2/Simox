#pragma once

#include <eigen3/Eigen/Core>

namespace simox::math
{

    /// Indicates whether the matrix is orthogonal, i.e. matrix * matrix.transpose = identity.
    template <typename Derived>
    bool
    is_matrix_orthogonal(const Eigen::MatrixBase<Derived>& matrix, float precision = 1e-6f)
    {
        return (matrix * matrix.transpose()).isIdentity(precision);
    }

    /// Compute the closest orthogonal matrix to the given matrix.
    /// (Note: All rotation matrices must be orthogonal.)
    Eigen::Matrix3f orthogonalize(const Eigen::Matrix3f& matrix);

    /// Orthogonolize the given matrix using Jacobi SVD decomposition.
    Eigen::Matrix3f orthogonalize_svd(const Eigen::Matrix3f& matrix);

    /// Orthogonolize the given matrix using Householder QR decomposition.
    Eigen::Matrix3f orthogonalize_qr(const Eigen::Matrix3f& matrix);


    /// Orthogonolize the orientation of the given pose, and sanitize its lower row.
    Eigen::Matrix4f orthogonalize_pose(const Eigen::Matrix4f& pose);

} // namespace simox::math
