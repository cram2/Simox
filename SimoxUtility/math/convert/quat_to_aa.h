#pragma once

#include <Eigen/Geometry>

namespace simox::math
{
    inline Eigen::AngleAxisf
    quat_to_aa(const Eigen::Quaternionf& q)
    {
        return Eigen::AngleAxisf{q.toRotationMatrix()};
    }

    inline void
    quat_to_aa(const Eigen::Quaternionf& q, Eigen::AngleAxisf& aa)
    {
        aa = quat_to_aa(q);
    }
} // namespace simox::math
