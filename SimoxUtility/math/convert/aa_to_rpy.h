#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "mat3f_to_rpy.h"

//out param version
namespace simox::math
{
    template <class D1>
    inline meta::enable_if_vec3<D1>
    aa_to_rpy(const Eigen::AngleAxisf& aa, Eigen::MatrixBase<D1>& rpy)
    {
        rpy = mat3f_to_rpy(aa.toRotationMatrix());
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2>
    aa_to_rpy(const Eigen::MatrixBase<D1>& ax, float ang, Eigen::MatrixBase<D2>& rpy)
    {
        aa_to_rpy(Eigen::AngleAxisf{ang, ax.normalized()}, rpy);
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2>
    aa_to_rpy(float ang, const Eigen::MatrixBase<D1>& ax, Eigen::MatrixBase<D2>& rpy)
    {
        aa_to_rpy(Eigen::AngleAxisf{ang, ax.normalized()}, rpy);
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2>
    aa_to_rpy(const Eigen::MatrixBase<D1>& ax, Eigen::MatrixBase<D2>& rpy)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            rpy.setZero();
        }
        aa_to_rpy(Eigen::AngleAxisf{ang, ax / ang}, rpy);
    }
} // namespace simox::math

//return version
namespace simox::math
{
    inline Eigen::Vector3f
    aa_to_rpy(const Eigen::AngleAxisf& aa)
    {
        return mat3f_to_rpy(aa.toRotationMatrix());
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Vector3f>
    aa_to_rpy(const Eigen::MatrixBase<D1>& ax, float ang)
    {
        return aa_to_rpy(Eigen::AngleAxisf{ang, ax.normalized()});
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Vector3f>
    aa_to_rpy(float ang, const Eigen::MatrixBase<D1>& ax)
    {
        return aa_to_rpy(Eigen::AngleAxisf{ang, ax.normalized()});
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Vector3f>
    aa_to_rpy(const Eigen::MatrixBase<D1>& ax)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            return Eigen::Vector3f::Zero();
        }
        return aa_to_rpy(Eigen::AngleAxisf{ang, ax / ang});
    }
} // namespace simox::math
