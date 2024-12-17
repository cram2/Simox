#pragma once

// STD/STL
#include <type_traits>

// Eigen
#include <Eigen/Core>

namespace simox::meta
{
    template <class T>
    struct is_eigen_array : std::false_type
    {
    };

    template <class ScalarType, int Rows, int Cols>
    struct is_eigen_array<Eigen::Array<ScalarType, Rows, Cols>> : std::true_type
    {
        static constexpr int rows = Rows;
        static constexpr int cols = Cols;
        using scalar_t = ScalarType;
        using eigen_t = Eigen::Array<ScalarType, Rows, Cols>;
    };

    template <class T>
    static constexpr bool is_eigen_array_v = is_eigen_array<T>::value;

    template <class T>
    static constexpr bool is_floating_point_eigen_array_v =
        is_eigen_array_v<T> && std::is_floating_point_v<typename is_eigen_array<T>::scalar_t>;
} // namespace simox::meta
