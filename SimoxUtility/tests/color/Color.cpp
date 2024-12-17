/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    RobotAPI::ArmarXObjects::FrameTracking
 * @author     Adrian Knobloch ( adrian dot knobloch at student dot kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#define BOOST_TEST_MODULE SimoxUtility / color / Color

#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/color/Color.h>

namespace
{

}

BOOST_AUTO_TEST_CASE(test_fromRGBA_int_vs_float)
{
    BOOST_CHECK_EQUAL(simox::Color(0, 0, 0), simox::Color(0.f, 0.f, 0.f));
    BOOST_CHECK_EQUAL(simox::Color(127, 127, 127), simox::Color(.5f, .5f, .5f));
    BOOST_CHECK_EQUAL(simox::Color(255, 255, 255), simox::Color(1.f, 1.f, 1.f));
}

BOOST_AUTO_TEST_CASE(test_fromRGBA_from_vector3)
{
    const Eigen::Vector3i color(0, 127, 255);
    const Eigen::Vector3f colorf(0, 0.5, 1.0);

    BOOST_CHECK_EQUAL(simox::Color(color), simox::Color(colorf));
}
