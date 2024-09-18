#pragma once

#include "Maths.h"

#include <Eigen/Core>
#include <limits>


namespace VirtualRobot::hemisphere
{

    /**
     * @brief Wrapper around `hemisphere::Joint` caching the results.
     */
    class CachedMaths
    {
    public:

        /**
         * @brief Recompute the maths if the given `actuatorsAngle` differ from
         * the stored `_actuators`.
         */
        void update(const Eigen::Vector2f& actuatorsAngle);

        /**
         * @brief Recompute the maths if the given `actuatorsAngle` differ from
         * the stored `_actuators`.
         */
        void update(const Eigen::Vector2d& actuatorsAngle);

    public:
        static constexpr double EQUALITY_PRECISION = 1e-6;

        /// The joint math.
        hemisphere::Maths maths;

    private:
        /// The actuator values that were used to compute the joint math.
        Eigen::Vector2d _actuators = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());

    };

}
