#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

//return version
namespace simox::math
{
    inline Eigen::Quaternionf
    aa_to_quat(const Eigen::AngleAxisf& aa)
    {
        return Eigen::Quaternionf{aa.toRotationMatrix()};
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Quaternionf>
    aa_to_quat(const Eigen::MatrixBase<D1>& ax, float ang)
    {
        return aa_to_quat(Eigen::AngleAxisf{ang, ax.normalized()});
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Quaternionf>
    aa_to_quat(float ang, const Eigen::MatrixBase<D1>& ax)
    {
        return aa_to_quat(Eigen::AngleAxisf{ang, ax.normalized()});
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Quaternionf>
    aa_to_quat(const Eigen::MatrixBase<D1>& ax)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            return Eigen::Quaternionf::Identity();
        }
        return aa_to_quat(Eigen::AngleAxisf{ang, ax / ang});
    }
} // namespace simox::math

//out param version
namespace simox::math
{
    inline void
    aa_to_quat(const Eigen::AngleAxisf& aa, Eigen::Quaternionf& q)
    {
        q = aa_to_quat(aa);
    }

    template <class D1>
    inline meta::enable_if_vec3<D1>
    aa_to_quat(const Eigen::MatrixBase<D1>& ax, float ang, Eigen::Quaternionf& q)
    {
        q = aa_to_quat(ax, ang);
    }

    template <class D1>
    inline meta::enable_if_vec3<D1>
    aa_to_quat(float ang, const Eigen::MatrixBase<D1>& ax, Eigen::Quaternionf& q)
    {
        q = aa_to_quat(ang, ax);
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1>
    inline meta::enable_if_vec3<D1>
    aa_to_quat(const Eigen::MatrixBase<D1>& ax, Eigen::Quaternionf& q)
    {
        q = aa_to_quat(ax);
    }
} // namespace simox::math
