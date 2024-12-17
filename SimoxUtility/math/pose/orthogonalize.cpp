#include "orthogonalize.h"

#include <Eigen/SVD>

#include "pose.h"

Eigen::Matrix3f
simox::math::orthogonalize(const Eigen::Matrix3f& matrix)
{
    return orthogonalize_svd(matrix);
}

Eigen::Matrix3f
simox::math::orthogonalize_svd(const Eigen::Matrix3f& matrix)
{
    auto svd = matrix.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f orth = svd.matrixU() * svd.matrixV().transpose();
    if (orth.determinant() >= 0)
    {
        return orth;
    }
    else
    {
        return -orth;
    }
}

Eigen::Matrix3f
simox::math::orthogonalize_qr(const Eigen::Matrix3f& matrix)
{
    auto householder = matrix.householderQr();
    Eigen::Matrix3f orth = householder.householderQ();

    // Upper right triangular matrix of matrixQR() is R matrix.
    // If a diagonal entry of R is negative, the corresponding column
    // in Q must be inverted.
    for (int i = 0; i < matrix.cols(); ++i)
    {
        if (householder.matrixQR().diagonal()(i) < 0)
        {
            orth.col(i) *= -1;
        }
    }
    return orth;
}

Eigen::Matrix4f
simox::math::orthogonalize_pose(const Eigen::Matrix4f& pose)
{
    Eigen::Matrix4f orth = pose;
    orientation(orth) = orthogonalize(orientation(orth).eval());
    orth.row(3) << 0, 0, 0, 1;
    return orth;
}
