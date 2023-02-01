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
        expressions.compute(p1, p2, lever, theta0);
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
            expressions.ex,
            expressions.ey,
            expressions.ez
        };
    }


    Eigen::Matrix3d Joint::getEndEffectorRotation() const
    {
        // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
        Eigen::Matrix3d ori;
        ori << expressions.exx, expressions.eyx, expressions.ezx,
               expressions.exy, expressions.eyy, expressions.ezy,
               expressions.exz, expressions.eyz, expressions.ezz;
        return ori;
    }


    Eigen::Matrix4d Joint::getEndEffectorTransform() const
    {
        return simox::math::pose(getEndEffectorTranslation(), getEndEffectorRotation());
    }


    Joint::Jacobian Joint::getJacobian() const
    {
        Joint::Jacobian jacobian;
        jacobian << expressions.jx1, expressions.jx2,
                    expressions.jy1, expressions.jy2,
                    expressions.jz1, expressions.jz2,
                    expressions.jrx1, expressions.jrx2,
                    expressions.jry1, expressions.jry2,
                    expressions.jrz1, expressions.jrz2;

        // Current state of constructing the orientational part.
        // ToDo: Do this with symbolic math inside `Expressions`.
        {
            const Eigen::Vector3d eefStateTrans = getEndEffectorTranslation();

            // Assume we move with (+1, +1) - this should cancel out.
            const Eigen::Vector2d actuatorVel = Eigen::Vector2d::Ones();
            const Eigen::Vector3d eefVelTrans = jacobian.block<3, 2>(0, 0) * actuatorVel;

            /*
             * The rotation axis is orthogonal to the vector from origin to the
             * EEF (eefStateTrans) and the movement direction (eefVelTrans).
             *
             * For the scaling, ask Cornelius. :)
             */
            const Eigen::Vector3d scaledRotAxis = eefStateTrans.cross(eefVelTrans)
                    / eefStateTrans.norm() * 2;

            for (int column = 0; column < 2; ++column)
            {
                /* This check should not be necessary since we are setting
                 * actuatorVel = (+1, +1) above.
                 * However, in order to avoid breaking changes in the future,
                 * I will keep it here.
                 */
                if (actuatorVel(column) != 0)
                {
                    jacobian.block<3, 1>(0, column) = scaledRotAxis / actuatorVel(column);
                }
                else
                {
                    jacobian.block<3, 1>(0, column).setZero();
                }
            }
        }

        return jacobian;
    }


    Joint::ActuatorPosition Joint::angleToPosition(const Joint::ActuatorAngle& alpha) const
    {
        return lever * Eigen::sin((alpha + Eigen::Vector2d::Constant(theta0)).array());
    }

}
