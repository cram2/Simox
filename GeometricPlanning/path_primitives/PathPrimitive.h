#pragma once

#include <Eigen/Geometry>

#include <VirtualRobot/math/AbstractFunctionR1R6.h>

#include <GeometricPlanning/types.h>

namespace simox::geometric_planning
{
    struct ParameterRange
    {
        float min;
        float max;

        bool isInRange(float t) const noexcept;
    };

    class PathPrimitive : virtual public ::math::AbstractFunctionR1R6
    {
    public:
        virtual ParameterRange parameterRange() const = 0;

        /**
         * @brief Projects a point onto the parametric model's manifold.
         *
         * @param pose
         * @return the parameter closest to the pose
         */
        virtual float parameter(const Pose& pose) const = 0;

        /**
         * @brief Projects a point onto the parametric model's manifold.
         *
         * @param pose
         * @return the progress in range [0,1]
         */
        float progress(const Pose& pose) const;
        float progress(float param) const;

        // math::AbstractFunctionR1R6 interface
        Eigen::Vector3f GetPosition(float t) override;
        Eigen::Vector3f GetPositionDerivative(float t) override;
        Eigen::Quaternionf GetOrientation(float t) override;
        Eigen::Vector3f GetOrientationDerivative(float t) override;

        // const variants of the base class' methods
        virtual Eigen::Vector3f getPosition(float t) const = 0;
        virtual Eigen::Vector3f getPositionDerivative(float t) const = 0;
        virtual Eigen::Quaternionf getOrientation(float t) const = 0;
        virtual Eigen::Vector3f getOrientationDerivative(float t) const = 0;

        Pose getPose(float t) const;

        float clampParameter(float t) const;

        ~PathPrimitive() override = default;
    };

} // namespace simox::geometric_planning
