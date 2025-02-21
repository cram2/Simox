#pragma once

#include <type_traits>
#include <vector>

#include <eigen3/Eigen/Core>

#include "Color.h"

namespace simox::color
{

    /**
     * The color look-up table used by PCL point cloud viewer.
     *
     * "Color lookup table consisting of 256 colors structured in a maximally
     *  discontinuous manner. Generated using the method of Glasbey et al.
     *  (see https://github.com/taketwo/glasbey)" [1]
     *
     * [1](https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/common/colors.h)
     */
    class GlasbeyLUT
    {
    public:
        /**
         * @brief Get a color from the lookup table with given ID.
         * The ID is automaticall wrapped if greater than `size()`.
         */
        static Color at(std::size_t id, int alpha = 255);

        /**
         * @brief Get a color from the lookup table with given ID (with float values).
         * The ID is automaticall wrapped if greater than `size()`.
         */
        static Eigen::Vector4f atf(std::size_t id, float alpha = 1.f);

        /**
         * @brief Get a color from the lookup table with given ID.
         * The ID is automaticall wrapped if greater than `size()`.
         */
        template <typename UIntT, std::enable_if_t<std::is_unsigned_v<UIntT>, int> = 0>
        static Color
        at(UIntT id, int alpha = 255)
        {
            return at(static_cast<std::size_t>(id), alpha);
        }

        /**
         * @brief Get a color from the lookup table with given ID.
         * The ID is automaticall wrapped if greater than `size()`.
         * If `id` is negative, its absolute value is used.
         */
        template <typename IntT, std::enable_if_t<std::is_signed_v<IntT>, int> = 0>
        static Color
        at(IntT id, int alpha = 255)
        {
            return at(static_cast<std::size_t>(id >= 0 ? id : std::abs(id)), alpha);
        }

        /**
         * @brief Get a color from the lookup table with given ID (with float values).
         * The ID is automaticall wrapped if greater than `size()`.
         */
        template <typename UIntT, std::enable_if_t<std::is_unsigned_v<UIntT>, int> = 0>
        static Eigen::Vector4f
        atf(UIntT id, float alpha = 1.f)
        {
            return atf(static_cast<std::size_t>(id), alpha);
        }

        /**
         * @brief Get a color from the lookup table with given ID (with float values).
         * The ID is automaticall wrapped if greater than `size()`.
         * If `id` is negative, its absolute value is used.
         */
        template <typename IntT, std::enable_if_t<std::is_signed_v<IntT>, int> = 0>
        static Eigen::Vector4f
        atf(IntT id, float alpha = 1.f)
        {
            return atf(static_cast<std::size_t>(id >= 0 ? id : std::abs(id)), alpha);
        }

        /// Get the number of colors in the lookup table.;
        static std::size_t size();

        /// Get the raw lookup table (flat).
        static const std::vector<unsigned char>& data();


    private:
        /// Private constructor.
        GlasbeyLUT() = default;
    };


} // namespace simox::color
