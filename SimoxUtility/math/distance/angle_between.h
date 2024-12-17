#pragma once

#include <cmath>

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    /**
     * @brief Angle between l and r on the plane defined by normal n
     * @param l vector one
     * @param r vector two
     * @param n normal vector
     */
    template <class D1, class D2, class D3>
    inline std::enable_if_t<simox::meta::is_vec3_v<D1> && simox::meta::is_vec3_v<D2> &&
                                simox::meta::is_vec3_v<D3>,
                            float>
    angle_between_vec3f_vec3f(const Eigen::MatrixBase<D1>& l,
                              const Eigen::MatrixBase<D2>& r,
                              const Eigen::MatrixBase<D3>& n)
    {
        const Eigen::Vector3f ln = l.normalized();
        const Eigen::Vector3f rn = r.normalized();
        const Eigen::Vector3f nn = n.normalized();
        return -std::atan2(rn.cross(ln).dot(nn), ln.dot(rn));
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, float>
    cos_angle_between_vec3f_vec3f(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return std::max(-1.f, std::min(1.f, l.normalized().dot(r.normalized())));
    }

    template <class D1, class D2>
    inline meta::enable_if_vec3_vec3<D1, D2, float>
    angle_between_vec3f_vec3f_abs(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return std::acos(cos_angle_between_vec3f_vec3f(l, r));
    }
} // namespace simox::math
