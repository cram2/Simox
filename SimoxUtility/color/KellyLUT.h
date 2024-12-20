/*
This file is part of MMM.

MMM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MMM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MMM.  If not, see <http://www.gnu.org/licenses/>.
*
* @package    MMM
* @author     Andre Meixner
* @copyright  2020 High Performance Humanoid Technologies (H2T, 255; Karlsruhe, Germany
*
*/

#pragma once

#include <set>
#include <vector>

#include "Color.h"

namespace simox::color
{

    /**
 * @brief 20 distinguishable colors by Kenneth L. Kelly but without black and white, e.g. usable for creating graphs
 */
    class KellyLUT
    {
    public:
        static const inline Color kelly_vivid_yellow = Color(0xFF, 0xB3, 0x00, 255);
        static const inline Color kelly_strong_purple = Color(0x80, 0x3E, 0x75, 255);
        static const inline Color kelly_vivid_orange = Color(0xFF, 0x68, 0x00, 255);
        static const inline Color kelly_very_light_blue = Color(0xA6, 0xBD, 0xD7, 255);
        static const inline Color kelly_vivid_red = Color(0xC1, 0x00, 0x20, 255);
        static const inline Color kelly_grayish_yellow = Color(0xCE, 0xA2, 0x62, 255);
        static const inline Color kelly_medium_gray = Color(0x81, 0x70, 0x66, 255);

        //The following will not be good for people with defective color vision
        static const inline Color kelly_vivid_green = Color(0x00, 0x7D, 0x34, 255);
        static const inline Color kelly_strong_purplish_pink = Color(0xF6, 0x76, 0x8E, 255);
        static const inline Color kelly_strong_blue = Color(0x00, 0x53, 0x8A, 255);
        static const inline Color kelly_yellowish_pink = Color(0xFF, 0x7A, 0x5C, 255);
        static const inline Color kelly_strong_violet = Color(0x53, 0x37, 0x7A, 255);
        static const inline Color kelly_vivid_orange_yellow = Color(0xFF, 0x8E, 0x00, 255);
        static const inline Color kelly_strong_purplish_red = Color(0xB3, 0x28, 0x51, 255);
        static const inline Color kelly_vivid_greenish_yellow = Color(0xF4, 0xC8, 0x00, 255);
        static const inline Color kelly_strong_reddish_brown = Color(0x7F, 0x18, 0x0D, 255);
        static const inline Color kelly_vivid_yellowish_green = Color(0x93, 0xAA, 0x00, 255);
        static const inline Color kelly_deep_yellowish_brown = Color(0x59, 0x33, 0x15, 255);
        static const inline Color kelly_vivid_reddish_orange = Color(0xF1, 0x3A, 0x13, 255);
        static const inline Color kelly_dark_olive_green = Color(0x23, 0x2C, 0x16, 255);

        static const inline std::vector<Color> KELLY_COLORS_COLOR_BLIND = {kelly_vivid_yellow,
                                                                           kelly_strong_purple,
                                                                           kelly_vivid_orange,
                                                                           kelly_very_light_blue,
                                                                           kelly_vivid_red,
                                                                           kelly_grayish_yellow,
                                                                           kelly_medium_gray};

        static const inline std::vector<Color> KELLY_COLORS = {kelly_vivid_yellow,
                                                               kelly_strong_purple,
                                                               kelly_vivid_orange,
                                                               kelly_very_light_blue,
                                                               kelly_vivid_red,
                                                               kelly_grayish_yellow,
                                                               kelly_medium_gray,
                                                               kelly_vivid_green,
                                                               kelly_strong_purplish_pink,
                                                               kelly_strong_blue,
                                                               kelly_strong_violet,
                                                               kelly_strong_violet,
                                                               kelly_vivid_orange_yellow,
                                                               kelly_strong_purplish_red,
                                                               kelly_vivid_greenish_yellow,
                                                               kelly_strong_reddish_brown,
                                                               kelly_vivid_yellowish_green,
                                                               kelly_deep_yellowish_brown,
                                                               kelly_vivid_reddish_orange,
                                                               kelly_dark_olive_green};

        /**
     * @brief Get a color from the lookup table with given ID.
     * The ID is automaticall wrapped if greater than `size()`.
     */
        static Color
        at(std::size_t id, int alpha = 255, bool color_blind = false)
        {
            id = id % size(color_blind);
            return data(color_blind).at(id).with_alpha(alpha);
        }

        /**
     * @brief Get a color from the lookup table with given ID (with float values).
     * The ID is automaticall wrapped if greater than `size()`.
     */
        static Eigen::Vector4f
        atf(std::size_t id, float alpha = 1.f, bool color_blind = false)
        {
            return at(id, to_byte(alpha), color_blind).to_vector4f();
        }

        /**
     * @brief Get a color from the lookup table with given ID.
     * The ID is automaticall wrapped if greater than `size()`.
     */
        template <typename UIntT, std::enable_if_t<std::is_unsigned_v<UIntT>, int> = 0>
        static Color
        at(UIntT id, int alpha = 255, bool color_blind = false)
        {
            return at(static_cast<std::size_t>(id), alpha, color_blind);
        }

        /**
     * @brief Get a color from the lookup table with given ID.
     * The ID is automaticall wrapped if greater than `size()`.
     * If `id` is negative, its absolute value is used.
     */
        template <typename IntT, std::enable_if_t<std::is_signed_v<IntT>, int> = 0>
        static Color
        at(IntT id, int alpha = 255, bool color_blind = false)
        {
            return at(static_cast<std::size_t>(id >= 0 ? id : std::abs(id)), alpha, color_blind);
        }

        /**
     * @brief Get a color from the lookup table with given ID (with float values).
     * The ID is automaticall wrapped if greater than `size()`.
     */
        template <typename UIntT, std::enable_if_t<std::is_unsigned_v<UIntT>, int> = 0>
        static Eigen::Vector4f
        atf(UIntT id, float alpha = 1.f, bool color_blind = false)
        {
            return atf(static_cast<std::size_t>(id), alpha, color_blind);
        }

        /**
     * @brief Get a color from the lookup table with given ID (with float values).
     * The ID is automaticall wrapped if greater than `size()`.
     * If `id` is negative, its absolute value is used.
     */
        template <typename IntT, std::enable_if_t<std::is_signed_v<IntT>, int> = 0>
        static Eigen::Vector4f
        atf(IntT id, float alpha = 1.f, bool color_blind = false)
        {
            return atf(static_cast<std::size_t>(id >= 0 ? id : std::abs(id)), alpha, color_blind);
        }

        /// Get the number of colors in the lookup table.;
        static std::size_t
        size(bool color_blind = false)
        {
            return data(color_blind).size();
        }

        static std::vector<Color>
        data(bool color_blind = false)
        {
            return color_blind ? KELLY_COLORS_COLOR_BLIND : KELLY_COLORS;
        }
    };

    /**
 * @brief An RGBA color, where each component is a byte in [0, 255]. Additionally contains an id denoting its position
 */
    struct KellyColor : public Color
    {
        KellyColor(size_t id = -1) : KellyColor(Color(), id)
        {
        }

        KellyColor(uint8_t r, uint8_t g, uint8_t b, size_t id) : Color(r, g, b), id(id)
        {
        }

        KellyColor(Color color, size_t id) : KellyColor(color.r, color.g, color.b, id)
        {
        }

        bool
        operator<(const KellyColor& other) const
        {
            return id < other.id;
        }

        size_t id;
    };

    /**
 * @brief Stack of 20 distinguishable colors by Kenneth L. Kelly but without black and white, e.g. usable for creating graphs
 */
    class KellyColorStack
    {
    public:
        /**
     * @brief Initializes a Stack of colors
     * @param color_blind 7 instead of 20 colors for visually impaired
     */
        KellyColorStack(bool color_blind = false) :
            colors(createColorStack(KellyLUT::data(color_blind)))
        {
        }

        /**
     * @brief Returns the next Kelly color and removes it from the stack, black color if stack is empty
     * @return The next color from the top of the stack
     */
        KellyColor
        next()
        {
            if (!colors.empty())
            {
                auto it = colors.begin();
                auto color = *it;
                colors.erase(it);
                return color;
            }
            else
                return KellyColor();
        }

        /**
     * @brief Puts a color back on the stack depending on its id
     */
        void
        putBack(KellyColor color)
        {
            colors.insert(color);
        }

        /**
     * @return Amount of colors left on the stack
     */
        std::size_t
        left()
        {
            return colors.size();
        }

        bool
        empty()
        {
            return colors.empty();
        }

    private:
        inline std::set<KellyColor>
        createColorStack(const std::vector<Color>& colors)
        {
            std::set<KellyColor> set;
            size_t index = 0;
            for (const Color& color : colors)
            {
                set.insert(KellyColor(color, index++));
            }
            return set;
        }

        std::set<KellyColor> colors;
    };

} // namespace simox::color
