/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_DeltaAngleTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/periodic/periodic_clamp.h>
#include <SimoxUtility/math/periodic/periodic_diff.h>
#include <SimoxUtility/math/periodic/periodic_mean.h>

BOOST_AUTO_TEST_CASE(test_periodic_clamp)
{
    static const float pi = float(M_PI);
    static const float prec = 1e-4f;

    for (int i = -2; i <= 2; ++i)
    {
        const float period = i * 2 * pi;

        BOOST_TEST_CONTEXT("Period i = " << i << ", i.e. shift = " << period)
        {
            BOOST_CHECK_SMALL(simox::math::periodic_clamp(0.f + period, 0.f, 2 * pi), prec);
            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(3.f + period, 0.f, 2 * pi), 3, prec);
            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(6.f + period, 0.f, 2 * pi), 6, prec);

            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(-3.f + period, -pi, pi), -3, prec);
            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(-1.f + period, -pi, pi), -1, prec);
            BOOST_CHECK_SMALL(simox::math::periodic_clamp(0.f + period, -pi, pi), prec);
            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(1.f + period, -pi, pi), 1, prec);
            BOOST_CHECK_CLOSE(simox::math::periodic_clamp(3.f + period, -pi, pi), 3, prec);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_periodic_diff)
{
    static const double pi = M_PI;
    static const double prec = 1e-4;

    BOOST_CHECK_SMALL(simox::math::periodic_diff(0, 0, 0, 2 * pi), prec);

    BOOST_CHECK_CLOSE(simox::math::periodic_diff(.5, -.5, 0, 2 * pi), 1, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_diff(-.5, .5, 0, 2 * pi), -1, prec);

    BOOST_CHECK_CLOSE(simox::math::periodic_diff(1, -1, 0, 2 * pi), 2, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_diff(-1, 1, 0, 2 * pi), -2, prec);

    BOOST_CHECK_CLOSE(simox::math::periodic_diff(2 * pi + 0.5, 2 * pi - 0.5, 0, 2 * pi), 1, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_diff(2 * pi - 0.5, 2 * pi + 0.5, 0, 2 * pi), -1, prec);

    BOOST_CHECK_CLOSE(simox::math::periodic_diff(2 * pi + 1, 2 * pi - 1, 0, 2 * pi), 2, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_diff(2 * pi - 1, 2 * pi + 1, 0, 2 * pi), -2, prec);


    for (int i = -2; i <= 2; ++i)
    {
        const double period = i * 2 * pi;

        BOOST_TEST_CONTEXT("Period i = " << i << ", i.e. shift = " << period)
        {
            for (int v = -5; v <= 5; ++v)
            {
                for (double diff : {-2, 1, 0, 1, 2})
                {
                    double a = v + diff + period;
                    double b = v + period;
                    BOOST_TEST_CONTEXT("Checking (" << a << " - " << b << ") == " << diff)
                    {
                        BOOST_CHECK_CLOSE(simox::math::periodic_diff(a, b, 0, 2 * pi), diff, prec);
                        BOOST_CHECK_CLOSE(simox::math::periodic_diff(a, b, -pi, pi), diff, prec);
                    }
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_periodic_mean_single_values)
{
    static const double pi = M_PI;
    static const double prec = 1e-4;

    BOOST_CHECK_SMALL(simox::math::periodic_mean({0}, 0, 2 * pi), prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_mean({1}, 0, 2 * pi), 1, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_mean({2}, 0, 2 * pi), 2, prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_mean({1 + 2 * pi}, 0, 2 * pi), 1, prec);

    BOOST_CHECK_SMALL(simox::math::periodic_mean({0}, 0., 360.), prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_mean({180}, 0., 360.), 180, prec);
    BOOST_CHECK_SMALL(simox::math::periodic_mean({360}, 0., 360.), prec);
    BOOST_CHECK_CLOSE(simox::math::periodic_mean({360 + 180}, 0., 360.), 180, prec);
}

BOOST_AUTO_TEST_CASE(test_periodic_mean)
{
    using namespace simox::math;

    static const double pi = M_PI;
    static const double prec = 1e-4;

    BOOST_CHECK_SMALL(
        periodic_diff(periodic_mean({-3, -2, -1, 0, 1, 2, 3}, -pi, pi), pi, 0, 2 * pi), prec);
    BOOST_CHECK_SMALL(
        periodic_diff(
            periodic_mean({-3 + pi, -2 + pi, -1 + pi, 0 + pi, 1 + pi, 2 + pi, 3 + pi}, 0, 2 * pi),
            0,
            0,
            2 * pi),
        prec);

    BOOST_CHECK_SMALL(periodic_diff(periodic_mean({0, 360}, 0., 360.), 0., 0., 360.), prec);
    BOOST_CHECK_SMALL(periodic_diff(periodic_mean({350, 10}, 0., 360.), 0., 0., 360.), prec);

    BOOST_CHECK_SMALL(periodic_diff(periodic_mean({170, 190}, 0., 360.), 180., 0., 360.), prec);
}
