#pragma once

#include "aa_to_mat4f.h"

//out patam version
namespace simox::math
{
    template <class D1>
    inline meta::enable_if_mat4<D1>
    pos_aa_to_mat4f(float x,
                    float y,
                    float z,
                    const Eigen::AngleAxisf& aa,
                    Eigen::MatrixBase<D1>& m4)
    {
        aa_to_mat4f(aa, m4);
        m4(0, 3) = x;
        m4(1, 3) = y;
        m4(2, 3) = z;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                    const Eigen::AngleAxisf& aa,
                    Eigen::MatrixBase<D2>& m4)
    {
        aa_to_mat4f(aa, m4);
        m4.template topRightCorner<3, 1>() = pos;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_aa_to_mat4f(float x,
                    float y,
                    float z,
                    const Eigen::MatrixBase<D1>& ax,
                    float ang,
                    Eigen::MatrixBase<D2>& m4)
    {
        pos_aa_to_mat4f(x, y, z, Eigen::AngleAxisf{ang, ax.normalized()}, m4);
    }

    template <class D1, class D2, class D3>
    inline meta::enable_if_vec3_vec3_mat4<D1, D2, D3>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                    const Eigen::MatrixBase<D2>& ax,
                    float ang,
                    Eigen::MatrixBase<D3>& m4)
    {
        pos_aa_to_mat4f(pos, Eigen::AngleAxisf{ang, ax.normalized()}, m4);
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_aa_to_mat4f(float x,
                    float y,
                    float z,
                    float ang,
                    const Eigen::MatrixBase<D1>& ax,
                    Eigen::MatrixBase<D2>& m4)
    {
        pos_aa_to_mat4f(x, y, z, Eigen::AngleAxisf{ang, ax.normalized()}, m4);
    }

    template <class D1, class D2, class D3>
    inline meta::enable_if_vec3_vec3_mat4<D1, D2, D3>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                    float ang,
                    const Eigen::MatrixBase<D2>& ax,
                    Eigen::MatrixBase<D3>& m4)
    {
        pos_aa_to_mat4f(pos, Eigen::AngleAxisf{ang, ax.normalized()}, m4);
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1, class D2>
    inline meta::enable_if_vec3_mat4<D1, D2>
    pos_aa_to_mat4f(float x,
                    float y,
                    float z,
                    const Eigen::MatrixBase<D1>& ax,
                    Eigen::MatrixBase<D2>& m4)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            pos_aa_to_mat4f(x, y, z, Eigen::AngleAxisf::Identity(), m4);
        }
        else
        {
            pos_aa_to_mat4f(x, y, z, Eigen::AngleAxisf{ang, ax / ang}, m4);
        }
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1, class D2, class D3>
    inline meta::enable_if_vec3_vec3_mat4<D1, D2, D3>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos,
                    const Eigen::MatrixBase<D2>& ax,
                    Eigen::MatrixBase<D3>& m4)
    {
        const auto ang = ax.norm();
        if (ang < 1e-5f)
        {
            pos_aa_to_mat4f(pos, Eigen::AngleAxisf::Identity(), m4);
        }
        else
        {
            pos_aa_to_mat4f(pos, Eigen::AngleAxisf{ang, ax / ang}, m4);
        }
    }
} // namespace simox::math

//return version
namespace simox::math
{
    inline Eigen::Matrix4f
    pos_aa_to_mat4f(float x, float y, float z, const Eigen::AngleAxisf& aa)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(x, y, z, aa, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::AngleAxisf& aa)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(pos, aa, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_aa_to_mat4f(float x, float y, float z, const Eigen::MatrixBase<D1>& ax, float ang)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(x, y, z, ax, ang, m4);
        return m4;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, Eigen::Matrix4f>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::MatrixBase<D2>& ax, float ang)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(pos, ax, ang, m4);
        return m4;
    }

    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_aa_to_mat4f(float x, float y, float z, float ang, const Eigen::MatrixBase<D1>& ax)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(x, y, z, ax, ang, m4);
        return m4;
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, Eigen::Matrix4f>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos, float ang, const Eigen::MatrixBase<D2>& ax)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(pos, ax, ang, m4);
        return m4;
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1>
    inline meta::enable_if_vec3<D1, Eigen::Matrix4f>
    pos_aa_to_mat4f(float x, float y, float z, const Eigen::MatrixBase<D1>& ax)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(x, y, z, ax, m4);
        return m4;
    }

    /// @brief assumes a premultiplied axis angle
    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, Eigen::Matrix4f>
    pos_aa_to_mat4f(const Eigen::MatrixBase<D1>& pos, const Eigen::MatrixBase<D2>& ax)
    {
        Eigen::Matrix4f m4;
        pos_aa_to_mat4f(pos, ax, m4);
        return m4;
    }
} // namespace simox::math
