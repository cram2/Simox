#include "Joint.h"

#include <cmath>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/pose/pose.h>


namespace VirtualRobot::hemisphere
{

    Joint::Joint() :
        Joint(1, simox::math::deg_to_rad(25.))
    {
    }


    Joint::Joint(double lever, double theta0)
    {
        this->setConstants(lever, theta0);
    }


    void Joint::setConstants(double lever, double theta0)
    {
        this->lever = lever;
        this->theta0 = theta0;
        this->radius = 2 * std::sin(theta0) * lever;

        this->limitHi =   simox::math::deg_to_rad(45 - 6.0);
        this->limitLo = - simox::math::deg_to_rad(45 - 14.0);
    }


    void Joint::computeFkOfPosition(double p1, double p2)
    {
        fk.compute(p1, p2, lever, theta0);
    }


    void Joint::computeFkOfPosition(const ActuatorPosition& p12)
    {
        computeFkOfPosition(p12(0), p12(1));
    }


    void Joint::computeFkOfAngle(const ActuatorAngle& alpha12)
    {
        computeFkOfPosition(angleToPosition(alpha12));
    }


    Eigen::Vector3d Joint::getEndEffectorTranslation() const
    {
        return Eigen::Vector3d {
            fk.ex,
            fk.ey,
            fk.ez
        };
    }


    Eigen::Matrix3d Joint::getEndEffectorRotation() const
    {
        // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
        Eigen::Matrix3d ori;
        ori << fk.exx, fk.eyx, fk.ezx,
               fk.exy, fk.eyy, fk.ezy,
               fk.exz, fk.eyz, fk.ezz;
        return ori;
    }


    Eigen::Matrix4d Joint::getEndEffectorTransform() const
    {
        return simox::math::pose(getEndEffectorTranslation(), getEndEffectorRotation());
    }


    Joint::Jacobian Joint::getJacobian() const
    {
        Joint::Jacobian jacobian;
        jacobian << fk.jx1, fk.jx2,
                    fk.jy1, fk.jy2,
                    fk.jz1, fk.jz2,
                    fk.jrx1, fk.jrx2,
                    fk.jry1, fk.jry2,
                    fk.jrz1, fk.jrz2;
        return jacobian;
    }


    Joint::ActuatorPosition Joint::angleToPosition(const Joint::ActuatorAngle& alpha) const
    {
        return lever * Eigen::sin((alpha + Eigen::Vector2d::Constant(theta0)).array());
    }

}
