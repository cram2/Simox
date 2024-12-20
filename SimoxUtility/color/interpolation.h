#pragma once

#include "Color.h"

namespace simox::color::interpol
{

    /// Interpolate linearly in RGB space.
    Color linear(float t, const Color& lhs, const Color& rhs);

    /// Interpolate linearly in HSV space. (`lhs` and `rhs` are expected to be RGB.)
    Color linear_hsv(float t, const Color& lhs, const Color& rhs);

} // namespace simox::color::interpol
