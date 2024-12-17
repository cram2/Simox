#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

//out param version
namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat3<D1>
    aa_to_mat3f(const Eigen::AngleAxisf& aa, Eigen::MatrixBase<D1>& mx)
    {
        mx = aa.toRotationMatrix();
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat3<D1, D2>
    aa_to_mat3f(const Eigen::MatrixBase<D1>& ax, float ang, Eigen::MatrixBase<D2>& mx)
    {
        aa_to_mat3f(Eigen::AngleAxisf{ang, ax.normalized()}, mx);
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat3<D1, D2>
    aa_to_mat3f(float ang, const Eigen::MatrixBase<D1>& ax, Eigen::MatrixBase<D2>& mx)
    {
        aa_to_mat3f(ax.normalized(), ang, mx);
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1, class D2>
    inline meta::enable_if_vec3_mat3<D1, D2>
    aa_to_mat3f(const Eigen::MatrixBase<D1>& ax, Eigen::MatrixBase<D2>& mx)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            mx.setIdentity();
        }
        else
        {
            aa_to_mat3f(Eigen::AngleAxisf{ang, ax / ang}, mx);
        }
    }
} // namespace simox::math

//return version
namespace simox::math
{
    inline Eigen::Matrix3f
    aa_to_mat3f(const Eigen::AngleAxisf& aa)
    {
        return aa.toRotationMatrix();
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix3f>
    aa_to_mat3f(const Eigen::MatrixBase<D1>& ax, float ang)
    {
        return aa_to_mat3f(Eigen::AngleAxisf{ang, ax});
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix3f>
    aa_to_mat3f(float ang, const Eigen::MatrixBase<D1>& ax)
    {
        return aa_to_mat3f(Eigen::AngleAxisf{ang, ax.normalized()});
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix3f>
    aa_to_mat3f(const Eigen::MatrixBase<D1>& ax)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            return Eigen::Matrix3f::Identity();
        }
        return aa_to_mat3f(Eigen::AngleAxisf{ang, ax / ang});
    }
} // namespace simox::math
