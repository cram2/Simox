#include "interpolation.h"

#include "hsv.h"

namespace simox::color
{

    Color
    interpol::linear(float t, const Color& lhs, const Color& rhs)
    {
        return Color(((1 - t) * lhs.to_vector4f() + t * rhs.to_vector4f()).eval());
    }

    Color
    linear_hsv(float t, const Color& lhs, const Color& rhs)
    {
        const Eigen::Vector3f lhs_hsv = rgb_to_hsv(lhs.to_vector3f());
        const Eigen::Vector3f rhs_hsv = rgb_to_hsv(rhs.to_vector3f());
        return Color(hsv_to_rgb(((1 - t) * lhs_hsv + t * rhs_hsv).eval()));
    }

} // namespace simox::color
