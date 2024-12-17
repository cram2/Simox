#pragma once

#include <Eigen/Core>

namespace simox::math
{

    /**
     * @brief Aligned box orientation and extents.
     * @see `align_box_orientation()`.
     */
    struct AlignedBox
    {
        /// The rotation to align the box orientation-
        Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

        /// The aligned orientation.
        Eigen::Matrix3f orientation = Eigen::Matrix3f::Identity();
        /// The aligned extents.
        Eigen::Vector3f extents = Eigen::Vector3f::Zero();
    };

    /**
     * @brief Align the given box orientation and extents to the canonic orientation.
     *
     * The axes of the aligned orientation O are positively aligned to the corresponding
     * axes of the canonic orientation C. That is, x_O points in the same direction
     * as x_C (same for y and z).
     *
     * The given extents are updated to keep the original extents.
     *
     * @param orientation The orientation to align.
     * @param extents The box extents.
     * @param canonicOrientation The canonic orientation (global by default).
     * @return The aligned box orientation and extents.
     */
    AlignedBox
    align_box_orientation(Eigen::Matrix3f orientation,
                          Eigen::Vector3f extents,
                          Eigen::Matrix3f canonicOrientation = Eigen::Matrix3f::Identity());


    /**
     * @brief Align the given box orientation to the canonic orientation.
     *
     * @param orientation The orientation to align.
     * @param canonicOrientation The canonic orientation (global by default).
     * @return The aligned box orientation.
     *
     * @see `align_box_orientation()`
     */
    Eigen::Matrix3f
    align_box_orientation(Eigen::Matrix3f orientation,
                          Eigen::Matrix3f canonicOrientation = Eigen::Matrix3f::Identity());


    /**
     * @brief Get a rotation matrix R aligning `orientation` O to the canonic orientation.
     *
     * @param orientation The orientation O to align.
     * @param canonicOrientation The canonic orientation (global by default).
     * @return A rotation matrix M so that O' = O * R is canonic.
     *
     * @see `align_box_orientation()`
     */
    Eigen::Matrix3f
    get_rotation_to_align_box(Eigen::Matrix3f orientation,
                              Eigen::Matrix3f canonicOrientation = Eigen::Matrix3f::Identity());

} // namespace simox::math
