#pragma once

#include "Maths.h"

#include <Eigen/Core>


namespace VirtualRobot::hemisphere
{

    /**
     * @brief Wrapper around `hemisphere::Joint` caching the results.
     */
    class CachedMaths
    {
    public:

        /**
         * @brief Recompute the maths if the given `actuators` differ from
         * the stored `actuators`.
         */
        void update(const Eigen::Vector2f& actuators);

    public:

        /// The actuator values that were used to compute the joint math.
        Eigen::Vector2f actuators = Eigen::Vector2f::Constant(std::numeric_limits<float>::min());

        /// The joint math.
        hemisphere::Maths maths;

    };

}
