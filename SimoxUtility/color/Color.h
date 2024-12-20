#pragma once

#include <iosfwd>
#include <type_traits>

#include <Eigen/Core>

#include "convert_byte_float.hpp"

namespace simox::color
{

    /**
     * @brief An RGBA color, where each component is a byte in [0, 255].
     */
    struct Color
    {
        // Attributes.

        /// Red in [0, 255].
        uint8_t r = 0;
        /// Green in [0, 255].
        uint8_t g = 0;
        /// Blue in [0, 255].
        uint8_t b = 0;
        /// Alpha in [0, 255], i.e. opacity (0: transparent, 255: visible).
        uint8_t a = 255;


        // Constructors.

        /// Construct a black color.
        Color() = default;

        /// Construct a color from RGBA.
        Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

        /// Construct a color from integer RGBA.
        template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
        Color(Int r, Int g, Int b, Int a = 255) :
            Color(to_byte(r), to_byte(g), to_byte(b), to_byte(a))
        {
        }

        /// Construct a color from float RGBA.
        template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
        Color(Float r, Float g, Float b, Float a = 1.0) :
            Color(to_byte(r), to_byte(g), to_byte(b), to_byte(a))
        {
        }

        /// Construct a color from an RGB vector.
        template <typename T>
        Color(Eigen::Matrix<T, 3, 1> vector3) : Color(vector3(0), vector3(1), vector3(2))
        {
        }

        /// Construct a color from an RGBA vector.
        template <typename T>
        Color(Eigen::Matrix<T, 4, 1> vector4) :
            Color(vector4(0), vector4(1), vector4(2), vector4(3))
        {
        }

        // Converters.

        Eigen::Vector3i to_vector3i() const;
        Eigen::Vector4i to_vector4i() const;
        Eigen::Vector3f to_vector3f() const;
        Eigen::Vector4f to_vector4f() const;

        /// Return a copy of this with the given alpha.
        template <typename IntOrFloat>
        Color
        with_alpha(IntOrFloat a) const
        {
            return Color(r, g, b, to_byte(a));
        }

        // Named colors.

        // Colorless

        static inline Color
        black(int a = 255)
        {
            return Color(0, 0, 0, a);
        }

        static inline Color
        white(int a = 255)
        {
            return Color(255, 255, 255, a);
        }

        static inline Color
        gray(int g = 128, int a = 255)
        {
            return Color(g, g, g, a);
        }

        // Primary colors

        static inline Color
        red(int r = 255, int a = 255)
        {
            return Color(r, 0, 0, a);
        }

        static inline Color
        green(int g = 255, int a = 255)
        {
            return Color(0, g, 0, a);
        }

        static inline Color
        blue(int b = 255, int a = 255)
        {
            return Color(0, 0, b, a);
        }

        // Secondary colors

        /// Green + Blue
        static inline Color
        cyan(int c = 255, int a = 255)
        {
            return Color(0, c, c, a);
        }

        /// Red + Green
        static inline Color
        yellow(int y = 255, int a = 255)
        {
            return Color(y, y, 0, a);
        }

        /// Red + Blue
        static inline Color
        magenta(int m = 255, int a = 255)
        {
            return Color(m, 0, m, a);
        }

        // 2:1 Mixed colors

        /// 2 Red + 1 Green
        static inline Color
        orange(int o = 255, int a = 255)
        {
            return Color(o, o / 2, 0, a);
        }

        /// 2 Red + 1 Blue
        static inline Color
        pink(int p = 255, int a = 255)
        {
            return Color(p, p / 2, 0, a);
        }

        /// 2 Green + 1 Red
        static inline Color
        lime(int l = 255, int a = 255)
        {
            return Color(l / 2, l, 0, a);
        }

        /// 2 Green + 1 Blue
        static inline Color
        turquoise(int t = 255, int a = 255)
        {
            return Color(0, t, t / 2, a);
        }

        /// 2 Blue + 1 Green
        static inline Color
        azure(int az = 255, int a = 255)
        {
            return Color(0, az / 2, az, a);
        }

        /// 2 Blue + 1 Red
        static inline Color
        purple(int p = 255, int a = 255)
        {
            return Color(0, p / 2, p, a);
        }

        // KIT colors (source: https://www.sek.kit.edu/downloads/2_Gestaltungsgrundlagen_Farben.pdf)

        static inline Color
        kit_green(int a = 255)
        {
            return Color(0, 150, 130, a);
        }

        static inline Color
        kit_blue(int a = 255)
        {
            return Color(70, 100, 170, a);
        }

        static inline Color
        kit_maygreen(int a = 255)
        {
            return Color(140, 182, 60, a);
        }

        static inline Color
        kit_yellow(int a = 255)
        {
            return Color(252, 229, 0, a);
        }

        static inline Color
        kit_orange(int a = 255)
        {
            return Color(223, 155, 27, a);
        }

        static inline Color
        kit_brown(int a = 255)
        {
            return Color(167, 130, 46, a);
        }

        static inline Color
        kit_red(int a = 255)
        {
            return Color(162, 34, 35, a);
        }

        static inline Color
        kit_purple(int a = 255)
        {
            return Color(163, 16, 124, a);
        }

        static inline Color
        kit_cyan(int a = 255)
        {
            return Color(35, 161, 224, a);
        }
    };

    std::ostream& operator<<(std::ostream& os, const Color& c);

    bool operator==(const Color& lhs, const Color& rhs);
    bool operator!=(const Color& lhs, const Color& rhs);
} // namespace simox::color

namespace simox
{
    // Expose color type as `simox::Color`.

    /// An RGBA color, where each component is a byte in [0, 255].
    using Color = color::Color;
} // namespace simox
