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

#define BOOST_TEST_MODULE SimoxUtility / color / ColorMap

#include <iostream>
#include <set>
#include <vector>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/random/choice.h>

namespace ChoiceTest
{

    struct Fixture
    {

        std::vector<std::string> vector{"a", "b", "c"};
        std::set<std::string> set{vector.begin(), vector.end()};

        Fixture()
        {
        }

        ~Fixture()
        {
        }
    };

} // namespace ChoiceTest


BOOST_FIXTURE_TEST_SUITE(ChoiceTest, ChoiceTest::Fixture)

BOOST_AUTO_TEST_CASE(test_choice_vector)
{
    std::set<std::string> allChosen;
    for (int i = 0; i < 1e5; ++i)
    {
        std::string chosen = simox::random::choice(vector);
        BOOST_CHECK(std::find(vector.begin(), vector.end(), chosen) != vector.end());

        allChosen.insert(chosen);
    }

    // All items must be hit at least once ... very likely.
    BOOST_CHECK_EQUAL(allChosen.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_choice_set)
{
    std::set<std::string> allChosen;
    for (int i = 0; i < 1e5; ++i)
    {
        std::string chosen = simox::random::choice(set);
        BOOST_CHECK(std::find(set.begin(), set.end(), chosen) != set.end());

        allChosen.insert(chosen);
    }

    // All items must be hit at least once ... very likely.
    BOOST_CHECK_EQUAL(allChosen.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_choice_empty)
{
    vector.clear();
    set.clear();
    BOOST_CHECK_THROW(simox::random::choice(vector), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::random::choice(set), simox::error::SimoxError);
}


BOOST_AUTO_TEST_SUITE_END()
