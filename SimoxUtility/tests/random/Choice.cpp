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

#define BOOST_TEST_MODULE SimoxUtility/color/ColorMap

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/random/choice.h>

#include <iostream>


namespace ChoiceTest
{

struct Fixture
{
    Fixture()
    {
    }
    ~Fixture()
    {
    }
};

}


BOOST_FIXTURE_TEST_SUITE(ChoiceTest, ChoiceTest::Fixture)


BOOST_AUTO_TEST_CASE(test_choice)
{
    std::vector<std::string> items { "a", "b", "c" };

    for (int i = 0; i < 100; ++i)
    {
        std::string chosen = simox::random::choice(items);
        BOOST_CHECK(std::find(items.begin(), items.end(), chosen) != items.end());
    }
}


BOOST_AUTO_TEST_CASE(test_choice_empty)
{
    std::vector<std::string> items;
    BOOST_CHECK_THROW(simox::random::choice(items), simox::error::SimoxError);
}


BOOST_AUTO_TEST_SUITE_END()
