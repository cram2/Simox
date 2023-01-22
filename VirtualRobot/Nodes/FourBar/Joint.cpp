#include "Joint.h"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/pose/pose.h>


namespace VirtualRobot::four_bar
{

    Joint::Joint(double theta0, const Dimensions& dimensions) : theta0(theta0), dims(dimensions)
    {
    }


    // void Joint::computeFkOfPosition(double p1, double p2)
    // {
    //     fk.compute(p1, p2, lever, theta0);
    // }


    // void Joint::computeFkOfPosition(const Eigen::Vector2d& p12)
    // {
    //     computeFkOfPosition(p12(0), p12(1));
    // }


    void
    Joint::computeFkOfAngle(const double theta)
    {
        // computeFkOfPosition(angleToPosition(alpha12));
        // transformation.setIdentity();

        // move from passive to active joint
        const Eigen::Translation3d passive_T_active_base(Eigen::Vector3d::UnitX() * dims.shank);

        // apply rotation of this active joint
        const Eigen::AngleAxisd active_base_T_eef{theta0 + theta, Eigen::Vector3d::UnitZ()};

        transformation = passive_T_active_base * active_base_T_eef;
    }


    Eigen::Vector3d
    Joint::getEndEffectorTranslation() const
    {
        return transformation.translation();
        // return Eigen::Vector3d{fk.ex, fk.ey, fk.ez};
    }


    Eigen::Matrix3d
    Joint::getEndEffectorRotation() const
    {
        return transformation.rotation();
        // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
        // Eigen::Matrix3d ori;
        // ori << fk.exx, fk.eyx, fk.ezx, fk.exy, fk.eyy, fk.ezy, fk.exz, fk.eyz, fk.ezz;
        // return ori;
    }


    Eigen::Matrix4d
    Joint::getEndEffectorTransform() const
    {
        return transformation.matrix();
    }


    Joint::Jacobian
    Joint::getJacobian() const
    {
        // FIXME implement
        Joint::Jacobian jacobian;
        jacobian << fk.jx1, fk.jx2, fk.jy1, fk.jy2, fk.jz1, fk.jz2, fk.jrx1, fk.jrx2, fk.jry1,
            fk.jry2, fk.jrz1, fk.jrz2;
        return jacobian;
    }

    // Eigen::Vector2d Joint::angleToPosition(const Eigen::Vector2d& alpha) const
    // {
    //     return lever * Eigen::sin((alpha + Eigen::Vector2d::Constant(theta0)).array());
    // }

} // namespace VirtualRobot::four_bar
