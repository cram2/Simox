#pragma once

#include <eigen3/Eigen/Core>

#include "Color.h"

namespace simox::color
{

    /**
     * @brief Convert an RGB color to HSV.
     *
     * Hue is measured in degrees, i.e. [0, 360].
     * All other channels (saturation, value, red, green, blue)
     * are in the interval [0, 1].
     *
     * @param rgb The RGB color with all channels in range [0, 1].
     * @return The HSV color in [0, 360] x [0, 1] x [0, 1].
     */
    Eigen::Vector3f rgb_to_hsv(const Eigen::Vector3f& rgb);


    /**
     * @brief Convert an HSV color to RGB
     *
     * @see `simox::color::rgb_to_hsv()` for the expected intervals.
     *
     * @param hsv The HSV color in [0, 360] x [0, 1] x [0, 1]
     * @return The RGB color with all channels in [0, 1].
     */
    Eigen::Vector3f hsv_to_rgb(const Eigen::Vector3f& hsv);

} // namespace simox::color
