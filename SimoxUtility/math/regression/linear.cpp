#include "linear.hpp"

namespace simox::math
{
    template class LinearRegression<1, float>;
    template class LinearRegression<1, double>;

    template class LinearRegression<2, float>;
    template class LinearRegression<2, double>;

    template class LinearRegression<3, float>;
    template class LinearRegression<3, double>;

    template class LinearRegression<4, float>;
    template class LinearRegression<4, double>;

} // namespace simox::math
