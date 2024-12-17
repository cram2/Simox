#pragma once

#include <type_traits>

#include <Eigen/Core>

namespace simox::color
{

    /// Convert an integral type to a byte in [0, 255].
    template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
    uint8_t
    to_byte(Int value)
    {
        return static_cast<uint8_t>(std::clamp(value, static_cast<Int>(0), static_cast<Int>(255)));
    }

    /// Convert a floating point type to a byte in [0, 255].
    template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    uint8_t
    to_byte(Float value)
    {
        return static_cast<uint8_t>(
            255 * std::clamp(value, static_cast<Float>(0.), static_cast<Float>(1.)));
    }

    /// Convert an integral point type to a float in [0.0, 1.0].
    template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
    float
    to_float(Int value)
    {
        return to_byte(value) / 255.f;
    }

    /// Convert a floating pont type to a float in [0.0, 1.0].
    template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    float
    to_float(Float value)
    {
        return std::clamp(value, static_cast<Float>(0.), static_cast<Float>(1.));
    }

    template <typename Scalar, int Rows, int Cols>
    Eigen::Matrix<int, Rows, Cols>
    to_byte(Eigen::Matrix<Scalar, Rows, Cols> vector)
    {
        return vector.unaryExpr([](Scalar s) { return int(simox::color::to_byte(s)); });
    }

    template <typename Scalar, int Rows, int Cols>
    Eigen::Matrix<float, Rows, Cols>
    to_float(Eigen::Matrix<Scalar, Rows, Cols> vector)
    {
        return vector.unaryExpr([](Scalar s) { return simox::color::to_float(s); });
    }

} // namespace simox::color
