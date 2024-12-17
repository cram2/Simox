#pragma once

#include <Eigen/Core>

namespace simox::math
{

    /// Invert the given pose in-place.
    void invert_pose(Eigen::Matrix4f& pose);

    /// Return the inverted of the given pose.
    template <typename Derived>
    Eigen::Matrix4f
    inverted_pose(const Eigen::MatrixBase<Derived>& pose)
    {
        Eigen::Matrix4f inv = pose;
        invert_pose(inv);
        return inv;
    }

} // namespace simox::math
