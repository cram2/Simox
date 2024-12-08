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

#include <SimoxUtility/color/ColorMap.h>
#include <SimoxUtility/color/cmaps.h>
#include <SimoxUtility/color/interpolation.h>



namespace ColorMapTest
{

}


BOOST_AUTO_TEST_SUITE(ColorMapTest)


BOOST_AUTO_TEST_CASE(test_at_empty)
{
    const simox::color::ColorMap cmap;
    BOOST_CHECK(cmap.empty());

    BOOST_CHECK_EQUAL(cmap.at(0), simox::Color::black());
    BOOST_CHECK_EQUAL(cmap.at(1), simox::Color::black());
    BOOST_CHECK_EQUAL(cmap.at(-100), simox::Color::black());
}


BOOST_AUTO_TEST_CASE(test_at_constant)
{
    const simox::Color color(0.0, 0.5, 1.0);
    const simox::color::ColorMap cmap1 { color };
    const simox::color::ColorMap cmap2 { { 0, color} };

    for (const auto& cmap : { cmap1, cmap2 })
    {
        BOOST_CHECK_EQUAL(cmap.size(), 1);

        BOOST_CHECK_EQUAL(cmap.at(-1), color);
        BOOST_CHECK_EQUAL(cmap.at( 0), color);
        BOOST_CHECK_EQUAL(cmap.at( 1), color);
    }
}


BOOST_AUTO_TEST_CASE(test_interpolate_linear)
{
    using namespace simox::color::interpol;

    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.0, 0.5, 1.0);

    BOOST_CHECK_EQUAL(linear(0, colorA, colorB), colorA);
    BOOST_CHECK_EQUAL(linear(1, colorA, colorB), colorB);

    BOOST_CHECK_EQUAL(linear(0.25, colorA, colorB), simox::Color(0.75, 0.5, 0.25));
    BOOST_CHECK_EQUAL(linear(0.50, colorA, colorB), simox::Color(0.50, 0.5, 0.50));
    BOOST_CHECK_EQUAL(linear(0.75, colorA, colorB), simox::Color(0.25, 0.5, 0.75));
}


BOOST_AUTO_TEST_CASE(test_two_elements)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.0, 0.5, 1.0);

    const simox::color::ColorMap cmap1 { colorA, colorB };
    const simox::color::ColorMap cmap2 { { 0, colorA }, { 1, colorB } };

    for (const auto& cmap : { cmap1, cmap2 })
    {
        BOOST_CHECK_EQUAL(cmap.at(-1), colorA);
        BOOST_CHECK_EQUAL(cmap.at( 0), colorA);
        BOOST_CHECK_EQUAL(cmap.at( 1), colorB);
        BOOST_CHECK_EQUAL(cmap.at( 2), colorB);

        BOOST_CHECK_EQUAL(cmap.at(0.25), simox::Color(0.75, 0.5, 0.25));
        BOOST_CHECK_EQUAL(cmap.at(0.50), simox::Color(0.50, 0.5, 0.50));
        BOOST_CHECK_EQUAL(cmap.at(0.75), simox::Color(0.25, 0.5, 0.75));
    }
}


BOOST_AUTO_TEST_CASE(test_three_elements)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.5, 0.5, 0.5);
    const simox::Color colorC(0.0, 0.5, 1.0);

    const simox::color::ColorMap cmap {
        { -1, colorA }, { 0, colorB }, { 2, colorC }
    };

    BOOST_CHECK_EQUAL(cmap.at(-2), colorA);
    BOOST_CHECK_EQUAL(cmap.at(-1), colorA);
    BOOST_CHECK_EQUAL(cmap.at( 0), colorB);
    BOOST_CHECK_EQUAL(cmap.at( 2), colorC);
    BOOST_CHECK_EQUAL(cmap.at( 3), colorC);

    BOOST_CHECK_EQUAL(cmap.at(-0.75), simox::Color(0.875, 0.5, 0.125));
    BOOST_CHECK_EQUAL(cmap.at(-0.50), simox::Color(0.750, 0.5, 0.250));
    BOOST_CHECK_EQUAL(cmap.at(-0.25), simox::Color(0.625, 0.5, 0.375));

    BOOST_CHECK_EQUAL(cmap.at(0.5), simox::Color(0.375, 0.5, 0.625));
    BOOST_CHECK_EQUAL(cmap.at(1.0), simox::Color(0.250, 0.5, 0.750));
    BOOST_CHECK_EQUAL(cmap.at(1.5), simox::Color(0.125, 0.5, 0.875));
}


BOOST_AUTO_TEST_CASE(test_vmin)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.0, 0.5, 1.0);

    simox::color::ColorMap cmap { { 1, colorA }, { 2, colorB } };
    cmap.set_vmin(0);

    BOOST_CHECK_EQUAL(cmap.at(-1), colorA);
    BOOST_CHECK_EQUAL(cmap.at( 0), colorA);

    BOOST_CHECK_EQUAL(cmap.at( 0.5), simox::Color(0.75, 0.5, 0.25));
    BOOST_CHECK_EQUAL(cmap.at( 1.0), simox::Color(0.50, 0.5, 0.50));
    BOOST_CHECK_EQUAL(cmap.at( 1.5), simox::Color(0.25, 0.5, 0.75));

    BOOST_CHECK_EQUAL(cmap.at( 2), colorB);
    BOOST_CHECK_EQUAL(cmap.at( 3), colorB);
}


BOOST_AUTO_TEST_CASE(test_vmax)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.0, 0.5, 1.0);

    simox::color::ColorMap cmap { { 1, colorA }, { 2, colorB } };
    cmap.set_vmax(3);

    BOOST_CHECK_EQUAL(cmap.at( 0), colorA);
    BOOST_CHECK_EQUAL(cmap.at( 1), colorA);

    BOOST_CHECK_EQUAL(cmap.at( 1.5), simox::Color(0.75, 0.5, 0.25));
    BOOST_CHECK_EQUAL(cmap.at( 2.0), simox::Color(0.50, 0.5, 0.50));
    BOOST_CHECK_EQUAL(cmap.at( 2.5), simox::Color(0.25, 0.5, 0.75));

    BOOST_CHECK_EQUAL(cmap.at( 3), colorB);
    BOOST_CHECK_EQUAL(cmap.at( 4), colorB);
}


BOOST_AUTO_TEST_CASE(test_vlimits)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.0, 0.5, 1.0);

    simox::color::ColorMap cmap { { 1, colorA }, { 2, colorB } };
    cmap.set_vlimits(100, 200);

    BOOST_CHECK_EQUAL(cmap.at(  99), colorA);
    BOOST_CHECK_EQUAL(cmap.at( 100), colorA);

    BOOST_CHECK_EQUAL(cmap.at( 125), simox::Color(0.75, 0.5, 0.25));
    BOOST_CHECK_EQUAL(cmap.at( 150), simox::Color(0.50, 0.5, 0.50));
    BOOST_CHECK_EQUAL(cmap.at( 175), simox::Color(0.25, 0.5, 0.75));

    BOOST_CHECK_EQUAL(cmap.at( 200), colorB);
    BOOST_CHECK_EQUAL(cmap.at( 201), colorB);
}


BOOST_AUTO_TEST_CASE(test_named_colormaps)
{
    simox::color::ColorMap cmap = simox::color::cmaps::viridis();
    BOOST_CHECK_EQUAL(cmap.name(), "viridis");

    cmap = simox::color::cmaps::get_named("viridis");
    BOOST_CHECK_EQUAL(cmap.name(), "viridis");

    namespace cmaps = simox::color::cmaps;
    BOOST_CHECK_GT(cmaps::Named::all().size(), 0);

    for (const auto& [name, cmap] : cmaps::Named::all())
    {
        BOOST_TEST_CONTEXT("ColorMap '" << name << "'")
        {
            BOOST_CHECK_EQUAL(cmap.name(), name);
            BOOST_CHECK_GE(cmap.size(), 2);
        }
    }
}


BOOST_AUTO_TEST_CASE(test_apply_vector)
{
    const simox::ColorMap cmap = simox::color::cmaps::viridis();

    const std::vector<float> vector = {0, 1, 2};
    const std::vector<simox::Color> cvector = cmap(vector);

    BOOST_CHECK_EQUAL(cvector.size(), vector.size());

    for (size_t i = 0; i < vector.size(); ++i)
    {
        BOOST_CHECK_EQUAL(cvector[i], cmap(vector[i]));
    }
}


BOOST_AUTO_TEST_CASE(test_apply_map)
{
    const simox::ColorMap colormap = simox::color::cmaps::viridis();

    const std::map<std::string, float> map = { {"0", 0.0} , {"1", 1.0}, {"2", 2.0} };
    const std::map<std::string, simox::Color> cmap = colormap(map);

    BOOST_CHECK_EQUAL(map.size(), cmap.size());

    for (const auto& [name, color] : cmap)
    {
        BOOST_CHECK_GT(map.count(name), 0);
        BOOST_CHECK_EQUAL(color, colormap(map.at(name)));
    }
}


BOOST_AUTO_TEST_CASE(test_reversed)
{
    const simox::Color colorA(1.0, 0.5, 0.0);
    const simox::Color colorB(0.5, 0.5, 0.5);
    const simox::Color colorC(0.0, 0.5, 1.0);

    const simox::color::ColorMap cmap {
        { -1, colorC }, { 0, colorB }, { 2, colorA }
    };

    const simox::color::ColorMap rev = cmap.reversed();

    BOOST_CHECK_EQUAL(rev.at(-2), colorA);
    BOOST_CHECK_EQUAL(rev.at(-1), colorA);
    BOOST_CHECK_EQUAL(rev.at( 0), colorB);
    BOOST_CHECK_EQUAL(rev.at( 2), colorC);
    BOOST_CHECK_EQUAL(rev.at( 3), colorC);

    BOOST_CHECK_EQUAL(rev.at(-0.75), simox::Color(0.875, 0.5, 0.125));
    BOOST_CHECK_EQUAL(rev.at(-0.50), simox::Color(0.750, 0.5, 0.250));
    BOOST_CHECK_EQUAL(rev.at(-0.25), simox::Color(0.625, 0.5, 0.375));

    BOOST_CHECK_EQUAL(rev.at(0.5), simox::Color(0.375, 0.5, 0.625));
    BOOST_CHECK_EQUAL(rev.at(1.0), simox::Color(0.250, 0.5, 0.750));
    BOOST_CHECK_EQUAL(rev.at(1.5), simox::Color(0.125, 0.5, 0.875));
}


BOOST_AUTO_TEST_SUITE_END()
