/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_RegressionTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/regression/linear.h>

namespace SimoxMathRegressionTest
{

    struct Fixture
    {
        const double prec = 1e-10;

        std::function<Eigen::Vector3d(double)> f = [](double x)
        {
            Eigen::Vector3d y;
            for (Eigen::Index i = 0; i < y.rows(); ++i)
            {
                y(i) = -(1 + i) + (2 * i) * x;
            }
            return y;
        };
        const std::vector<double> xs{-1, 0, 2};
        const std::vector<Eigen::Vector3d> ys{f(xs[0]), f(xs[1]), f(xs[2])};
    };
} // namespace SimoxMathRegressionTest


BOOST_FIXTURE_TEST_SUITE(SimoxMathRegressionTest, Fixture)

BOOST_AUTO_TEST_CASE(test_linear_regression_3d_fit_and_predict)
{
    using simox::math::LinearRegression3d;

    // Fit

    const LinearRegression3d regression = LinearRegression3d::Fit(xs, ys);

    BOOST_TEST_MESSAGE("Regression: " << regression);

    BOOST_CHECK_CLOSE(regression.coefficients(0, 0), -(1 + 0), prec);
    BOOST_CHECK_CLOSE(regression.coefficients(1, 0), -(1 + 1), prec);
    BOOST_CHECK_CLOSE(regression.coefficients(2, 0), -(1 + 2), prec);

    BOOST_CHECK_CLOSE(regression.coefficients(0, 1), (2 * 0), prec);
    BOOST_CHECK_CLOSE(regression.coefficients(1, 1), (2 * 1), prec);
    BOOST_CHECK_CLOSE(regression.coefficients(2, 1), (2 * 2), prec);


    // Predict

    BOOST_CHECK_LE((regression.predict(xs[0]) - ys[0]).norm(), prec);
    BOOST_CHECK_LE((regression.predict(xs[1]) - ys[1]).norm(), prec);
    BOOST_CHECK_LE((regression.predict(xs[2]) - ys[2]).norm(), prec);
}

BOOST_AUTO_TEST_CASE(test_linear_regression_3d_fit_and_predict_with_input_offset)
{
    using simox::math::LinearRegression3d;

    const bool inputOffset = true;
    const LinearRegression3d regression = LinearRegression3d::Fit(xs, ys, inputOffset);

    BOOST_TEST_MESSAGE("Regression: " << regression);
    BOOST_CHECK_EQUAL(regression.inputOffset, -xs[0]);

    // Coefficients are different now, but prediction should be the same.

    BOOST_CHECK_LE((regression.predict(xs[0]) - ys[0]).norm(), prec);
    BOOST_CHECK_LE((regression.predict(xs[1]) - ys[1]).norm(), prec);
    BOOST_CHECK_LE((regression.predict(xs[2]) - ys[2]).norm(), prec);
}


BOOST_AUTO_TEST_SUITE_END()
