#pragma once

#include <Eigen/Core>

#include "Expressions.h"


namespace VirtualRobot::hemisphere
{

    /**
     * @brief Hemisphere joint mathematics for FK and IK.
     *
     * This class is a wrapper about `Expressions`, which contains the actual
     * math code that was generated from Python sympy expressions
     * using `python/hemisphere-joint-demo/hemisphere_joint_demo/sympy_to_code.py`.
     */
    class Joint
    {
    public:

        using ActuatorPosition = Eigen::Vector2d;
        using ActuatorAngle = Eigen::Vector2d;
        using Jacobian = Eigen::Matrix<double, 6, 2>;

    public:

        Joint();
        Joint(double lever, double theta0);


        void setConstants(double lever, double theta0);


        void computeFkOfAngle(const ActuatorAngle& alpha12);

        void computeFkOfPosition(const ActuatorPosition& p12);
        void computeFkOfPosition(double p1, double p2);


        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;

        Jacobian getJacobian() const;

        ActuatorPosition angleToPosition(const ActuatorAngle& alpha) const;


    public:

        double lever = 0;
        double theta0 = 0;
        double radius = 0;

        double limitLo = 0;
        double limitHi = 0;


        Expressions fk;

    };

}
