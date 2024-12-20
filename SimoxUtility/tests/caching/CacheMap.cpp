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
#include <unordered_map>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/caching/CacheMap.h>

namespace CacheMapTest
{

    struct Fixture
    {
        Fixture()
        {
        }

        ~Fixture()
        {
        }

        template <class CacheT>
        void
        test_cache()
        {
            test_cache_with_fetchFn<CacheT>();
            test_cache_no_fetchFn_on_construction<CacheT>();
        }

        template <class CacheT>
        void
        test_cache_with_fetchFn()
        {
            auto fetchFn = [](int i) { return std::to_string(i); };

            CacheT cache(fetchFn);

            BOOST_CHECK(cache.empty());
            BOOST_CHECK_EQUAL(cache.size(), 0);

            BOOST_CHECK_EQUAL(cache.get(1), "1");
            BOOST_CHECK_EQUAL(cache.size(), 1);

            BOOST_CHECK_EQUAL(cache.get(1), "1");
            BOOST_CHECK_EQUAL(cache.size(), 1);

            BOOST_CHECK_EQUAL(cache.get(2), "2");
            BOOST_CHECK_EQUAL(cache.size(), 2);

            BOOST_CHECK_EQUAL(cache.get(2), "2");
            BOOST_CHECK_EQUAL(cache.size(), 2);

            BOOST_CHECK(cache.contains(1));
            BOOST_CHECK(cache.contains(2));
            BOOST_CHECK(!cache.contains(3));

            // Iterators are not printable => not using _EQUAL/_NE
            BOOST_CHECK(cache.find(1) != cache.end());
            BOOST_CHECK(cache.find(2) != cache.end());
            BOOST_CHECK(cache.find(3) == cache.end());


            // Test iteration
            BOOST_REQUIRE_GT(cache.size(), 0); // Otherwise, next test is meaningless.
            for (const auto& [key, value] : cache)
            {
                BOOST_CHECK_EQUAL(fetchFn(key), value);
            }


            // Resetting fetch function

            auto fetchFn2 = [](int i) { return std::to_string(-i); };

            const bool clear = true;
            cache.setFetchFn(fetchFn2, clear);
            BOOST_CHECK_EQUAL(cache.size(), 0);
            BOOST_CHECK(cache.empty());
            BOOST_CHECK(!cache.contains(1));
            BOOST_CHECK(!cache.contains(2));

            BOOST_CHECK_EQUAL(cache.get(1), fetchFn2(1));
            BOOST_CHECK_EQUAL(cache.get(3), fetchFn2(3));
        }

        template <class CacheT>
        void
        test_cache_no_fetchFn_on_construction()
        {
            auto fetchFn = [](int i) { return std::to_string(i); };

            CacheT cache;

            BOOST_CHECK_THROW(cache.get(1), simox::error::SimoxError);
            BOOST_CHECK_EQUAL(cache.get(1, fetchFn), fetchFn(1));

            cache.setFetchFn(fetchFn);
            BOOST_CHECK_EQUAL(cache.get(2, fetchFn), fetchFn(2));
        }
    };

} // namespace CacheMapTest


BOOST_FIXTURE_TEST_SUITE(CacheMapTest, CacheMapTest::Fixture)

BOOST_AUTO_TEST_CASE(test_default_map)
{
    test_cache<simox::caching::CacheMap<int, std::string>>();
}

BOOST_AUTO_TEST_CASE(test_map)
{
    test_cache<simox::caching::CacheMap<int, std::string, std::map>>();
}

BOOST_AUTO_TEST_CASE(test_unordered_map)
{
    test_cache<simox::caching::CacheMap<int, std::string, std::unordered_map>>();
}


BOOST_AUTO_TEST_SUITE_END()
