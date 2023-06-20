#include "Maths.h"

#include <cmath>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/pose/pose.h>

namespace VirtualRobot::hemisphere
{

    Maths::Maths() : Maths(1, simox::math::deg_to_rad(25.))
    {
    }

    Maths::Maths(double lever, double theta0)
    {
        this->setConstants(lever, theta0, std::nullopt, std::nullopt);
    }

    void
    Maths::setConstants(double lever,
                        double theta0,
                        std::optional<double> limitLo,
                        std::optional<double> limitHi)
    {
        this->lever = lever;
        this->theta0Rad = theta0;
        this->radiusOfRollingSpheres = std::sin(theta0) * lever;

        this->limitHi = simox::math::deg_to_rad(limitHi.has_value() ? limitHi.value() : (45 - 6.0));
        this->limitLo =
            simox::math::deg_to_rad(limitLo.has_value() ? limitLo.value() : -(45 - 14.0));
    }

    void
    Maths::computeFkOfPosition(double p1, double p2)
    {
        expressions.compute(p1, p2, lever, theta0Rad);
    }

    void
    Maths::computeFkOfPosition(const ActuatorPosition& p12)
    {
        computeFkOfPosition(p12(0), p12(1));
    }

    void
    Maths::computeFkOfAngle(const ActuatorAngle& alpha12)
    {
        computeFkOfPosition(angleToPosition(alpha12));
    }

    Eigen::Vector3d
    Maths::getEndEffectorTranslation() const
    {
        return Eigen::Vector3d{expressions.ex, expressions.ey, expressions.ez};
    }

    Eigen::Matrix3d
    Maths::getEndEffectorRotation() const
    {
        // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
        Eigen::Matrix3d ori;
        ori << expressions.exx, expressions.eyx, expressions.ezx, expressions.exy, expressions.eyy,
            expressions.ezy, expressions.exz, expressions.eyz, expressions.ezz;
        return ori;
    }

    Eigen::Matrix4d
    Maths::getEndEffectorTransform() const
    {
        return simox::math::pose(getEndEffectorTranslation(), getEndEffectorRotation());
    }

    Maths::Jacobian
    Maths::getJacobian() const
    {
        Maths::Jacobian jacobian;
        jacobian << expressions.jx1, expressions.jx2, expressions.jy1, expressions.jy2,
            expressions.jz1, expressions.jz2, expressions.jrx1, expressions.jrx2, expressions.jry1,
            expressions.jry2, expressions.jrz1, expressions.jrz2;

        // Current state of constructing the orientational part.
        // ToDo: Do this with symbolic math inside `Expressions`.
        {
            const Eigen::Vector3d eefStateTrans = getEndEffectorTranslation();

            for (int column = 0; column < 2; ++column)
            {
                // Imagine we apply (+1, 0) / (0, +1) actuator velocities.
                const Eigen::Vector3d eefVelTrans = jacobian.block<3, 1>(0, column); // * 1.0

                /*
                 * The rotation axis is orthogonal to the vector from origin to the
                 * EEF (eefStateTrans) and the movement direction (eefVelTrans).
                 *
                 * For the scaling, ask Cornelius. :)
                 */
                const Eigen::Vector3d scaledRotAxis =
                    eefStateTrans.cross(eefVelTrans) / eefStateTrans.norm() * 2;

                jacobian.block<3, 1>(3, column) = scaledRotAxis; // / 1.0;
            }
        }

        return jacobian;
    }

    Maths::ActuatorPosition
    Maths::angleToPosition(const Maths::ActuatorAngle& alpha) const
    {
        return lever * Eigen::sin((alpha + Eigen::Vector2d::Constant(theta0Rad)).array());
    }

} // namespace VirtualRobot::hemisphere
