#pragma once

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"
#include "SimoxUtility/math/periodic/periodic_clamp.h"

//3f
namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat3_mat3<D1, D2, float>
    delta_angle(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return std::abs(
            periodic_clamp<float>(Eigen::AngleAxisf{l * r.transpose()}.angle(), -M_PI, M_PI));
    }
} // namespace simox::math

//4f
namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat4_mat4<D1, D2, float>
    delta_angle(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return delta_angle(l.template topLeftCorner<3, 3>(), r.template topLeftCorner<3, 3>());
    }
} // namespace simox::math

//q
namespace simox::math
{
    inline float
    delta_angle(const Eigen::Quaternionf& l, const Eigen::Quaternionf& r)
    {
        return delta_angle(l.toRotationMatrix(), r.toRotationMatrix());
    }
} // namespace simox::math

//aa
namespace simox::math
{
    inline float
    delta_angle(const Eigen::AngleAxisf& l, const Eigen::AngleAxisf& r)
    {
        return delta_angle(l.toRotationMatrix(), r.toRotationMatrix());
    }
} // namespace simox::math

//3f 4f
namespace simox::math
{
    template <class D1, class D2>
    inline meta::enable_if_mat4_mat3<D1, D2, float>
    delta_angle(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return delta_angle(l.template topLeftCorner<3, 3>(), r);
    }

    template <class D1, class D2>
    inline meta::enable_if_mat3_mat4<D1, D2, float>
    delta_angle(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return delta_angle(l, r.template topLeftCorner<3, 3>());
    }
} // namespace simox::math

// q 3f
namespace simox::math
{
    template <class D>
    inline meta::enable_if_mat3<D, float>
    delta_angle(const Eigen::Quaternionf& l, const Eigen::MatrixBase<D>& r)
    {
        return delta_angle(l.toRotationMatrix(), r);
    }

    template <class D>
    inline meta::enable_if_mat3<D, float>
    delta_angle(const Eigen::MatrixBase<D>& l, const Eigen::Quaternionf& r)
    {
        return delta_angle(l, r.toRotationMatrix());
    }
} // namespace simox::math

// q 4f
namespace simox::math
{
    template <class D>
    inline meta::enable_if_mat4<D, float>
    delta_angle(const Eigen::Quaternionf& l, const Eigen::MatrixBase<D>& r)
    {
        return delta_angle(l.toRotationMatrix(), r.template topLeftCorner<3, 3>());
    }

    template <class D>
    inline meta::enable_if_mat4<D, float>
    delta_angle(const Eigen::MatrixBase<D>& l, const Eigen::Quaternionf& r)
    {
        return delta_angle(l.template topLeftCorner<3, 3>(), r.toRotationMatrix());
    }
} // namespace simox::math

// aa 3f
namespace simox::math
{
    template <class D>
    inline meta::enable_if_mat3<D, float>
    delta_angle(const Eigen::AngleAxisf& l, const Eigen::MatrixBase<D>& r)
    {
        return delta_angle(l.toRotationMatrix(), r);
    }

    template <class D>
    inline meta::enable_if_mat3<D, float>
    delta_angle(const Eigen::MatrixBase<D>& l, const Eigen::AngleAxisf& r)
    {
        return delta_angle(l, r.toRotationMatrix());
    }
} // namespace simox::math

// aa 4f
namespace simox::math
{
    template <class D>
    inline meta::enable_if_mat4<D, float>
    delta_angle(const Eigen::AngleAxisf& l, const Eigen::MatrixBase<D>& r)
    {
        return delta_angle(l.toRotationMatrix(), r.template topLeftCorner<3, 3>());
    }

    template <class D>
    inline meta::enable_if_mat4<D, float>
    delta_angle(const Eigen::MatrixBase<D>& l, const Eigen::AngleAxisf& r)
    {
        return delta_angle(l.template topLeftCorner<3, 3>(), r.toRotationMatrix());
    }
} // namespace simox::math

// q aa
namespace simox::math
{
    inline float
    delta_angle(const Eigen::Quaternionf& l, const Eigen::AngleAxisf& r)
    {
        return delta_angle(l.toRotationMatrix(), r.toRotationMatrix());
    }

    inline float
    delta_angle(const Eigen::AngleAxisf& l, const Eigen::Quaternionf& r)
    {
        return delta_angle(l.toRotationMatrix(), r.toRotationMatrix());
    }
} // namespace simox::math
