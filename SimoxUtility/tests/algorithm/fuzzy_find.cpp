/**
* @package    SimoxUtility
* @author     Andre Meixner
* @copyright  2020 Andre Meixner
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / fuzzy_find

#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/fuzzy_find.h>

void
search(auto name, auto& cont, auto key, auto acc, auto expected)
{
    const auto found = simox::alg::fuzzy_find(cont, key, acc);
    const auto dist = std::distance(cont.begin(), found);
    BOOST_CHECK(dist == expected);
    if (dist != expected)
    {
        std::cout << "    ERROR " << name << ' ' << key << " (" << acc << ") " << dist
                  << " (is end = " << (found == cont.end()) << ")\n";
    }
}

BOOST_AUTO_TEST_CASE(fuzzy_find)
{
    std::set<float> set{0, 1, 2, 3, 4};
    std::map<float, float> map{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}};
    std::set<float> const cset{0, 1, 2, 3, 4};
    std::map<float, float> const cmap{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}};

    const auto test = [&](auto k, auto a, auto e)
    {
        std::cout << "expect " << e << '\n';
        search(" map", map, k, a, e);
        search("cmap", cmap, k, a, e);
        search(" set", set, k, a, e);
        search("cset", cset, k, a, e);
    };

    test(-0.1, 0, 5);
    test(-0.1, 0.09, 5);
    test(-0.1, 0.11, 0);

    test(0, 0, 0);

    test(1, 0, 1);

    test(1.1, 0, 5);
    test(1.1, 0.09, 5);
    test(1.1, 0.11, 1);

    test(1.5, 0, 5);
    test(1.5, 0.49, 5);
    test(1.5, 0.51, 1);

    test(1.9, 0, 5);
    test(1.9, 0.09, 5);
    test(1.9, 0.11, 2);

    test(4.1, 0, 5);
    test(4.1, 0.09, 5);
    test(4.1, 0.11, 4);
}
