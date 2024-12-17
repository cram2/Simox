#pragma once

#include "linear.hpp"

namespace simox::math
{

    using LinearRegression1f = LinearRegression<1, float>;
    using LinearRegression2f = LinearRegression<2, float>;
    using LinearRegression3f = LinearRegression<3, float>;
    using LinearRegression4f = LinearRegression<4, float>;

    using LinearRegression1d = LinearRegression<1, double>;
    using LinearRegression2d = LinearRegression<2, double>;
    using LinearRegression3d = LinearRegression<3, double>;
    using LinearRegression4d = LinearRegression<4, double>;

} // namespace simox::math
