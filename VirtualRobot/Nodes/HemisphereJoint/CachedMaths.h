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
        inline void update(const Eigen::Vector2f& actuatorsAngle)
        {
            update(actuatorsAngle.cast<double>().eval());
        }

        /**
         * @brief Recompute the maths if the given `actuatorsAngle` differ from
         * the stored `_actuators`.
         */
        void update(const Eigen::Vector2d& actuatorsAngle);

    public:
        /// The joint math.
        hemisphere::Maths maths;

    private:
        static constexpr double EQUALITY_PRECISION = 1e-6;

        /// The actuator values that were used to compute the joint math.
        Eigen::Vector2d _actuators = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());

    };

}
