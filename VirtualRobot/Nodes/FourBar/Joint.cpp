#include "Joint.h"

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/pose/pose.h>


namespace VirtualRobot::four_bar
{

    Joint::Joint(const Dimensions& dimensions) : dims(dimensions)
    {
    }


    Eigen::Isometry3d
    Joint::computeFk(const double theta) const
    {
        // move from passive to active joint
        const Eigen::Translation3d passive_T_active_base(Eigen::Vector3d::UnitX() * dims.shank);

        // apply rotation of this active joint
        const Eigen::AngleAxisd active_base_T_eef{theta, Eigen::Vector3d::UnitZ()};

        return passive_T_active_base * active_base_T_eef;
    }

    Eigen::Isometry3d
    Joint::computeFkCombined(const double theta) const
    {
        // apply rotation of passive joint
        const Eigen::AngleAxisd passive_base_T_passive{psi(theta), -Eigen::Vector3d::UnitZ()};

        // move from passive to active joint
        const Eigen::Translation3d passive_T_active_base(Eigen::Vector3d::UnitX() * dims.shank);

        // apply rotation of this active joint
        const Eigen::AngleAxisd active_base_T_eef{theta, Eigen::Vector3d::UnitZ()};

        return passive_base_T_passive * passive_T_active_base * active_base_T_eef;
    }


    Joint::Jacobian
    Joint::getJacobian(const double theta, const Eigen::Vector3d& base_P_eef) const
    {
        Joint::Jacobian jacobian;

        // define helpers
        const double k1 = dims.k1();
        const double k2 = dims.k2();
        const double k3 = dims.k3();

        const double cosTheta = std::cos(theta);
        const double sinTheta = std::sin(theta);

        const double A = k1 * cosTheta + k2 + k3 + cosTheta; // C.34
        const double B = -2 * sinTheta; // C.35
        const double C = k1 * cosTheta - k2 + k3 - cosTheta; // C.36
        const double D = std::sqrt(B * B - 4 * A * C);

        const double psi = 2 * std::atan((-B + D) / (2 * A)); // C.39
        const double cosPsi = std::cos(psi);
        const double sinPsi = std::sin(psi);

        const Eigen::Isometry3d base_T_active = computeFkCombined(theta);

        const Eigen::Vector3d active_P_eef = base_T_active.inverse() * base_P_eef;

        const double& x = active_P_eef.x();
        const double& y = active_P_eef.y();

        //
        constexpr auto squared = [](const double t) { return t * t; };

        const double dD_dtheta =
            2 * (squared(k1) * cosTheta + k1 * k3 - k2) * sinTheta /
            std::sqrt(-squared(k1) * squared(cosTheta) - 2 * k1 * k3 * cosTheta + squared(k2) +
                      2 * k2 * cosTheta - squared(k3) + 1);


        const double dpsi_dtheta =
            4 *
            ((k1 + 1) * (D + 2 * sinTheta) * sinTheta +
             (2 * cosTheta + dD_dtheta) * (k1 * cosTheta + k2 + k3 + cosTheta)) /
            (squared(D + 2 * sinTheta) + 4 * squared(k1 * cosTheta + k2 + k3 + cosTheta));

        const double dpsi_dtheta_m1 = dpsi_dtheta - 1;
        const double sinThetaMPsi = std::sin(theta - psi);
        const double cosThetaMPsi = std::cos(theta - psi);

        const double dx_dtheta = -dims.shank * sinPsi * dpsi_dtheta +
                                 x * dpsi_dtheta_m1 * sinThetaMPsi +
                                 y * dpsi_dtheta_m1 * cosThetaMPsi;
        const double dy_dtheta = -dims.shank * cosPsi * dpsi_dtheta -
                                 x * dpsi_dtheta_m1 * cosThetaMPsi +
                                 y * dpsi_dtheta_m1 * sinThetaMPsi;

        const double dgamma_dtheta = 1 - dpsi_dtheta;

        jacobian(0) = dx_dtheta;
        jacobian(1) = dy_dtheta;
        jacobian(2) = dgamma_dtheta;

        return jacobian;
    }

    double
    Joint::psi(const double theta) const
    {
        const double k1 = dims.k1();
        const double k2 = dims.k2();
        const double k3 = dims.k3();

        const double cosTheta = std::cos(theta);
        const double sinTheta = std::sin(theta);

        const double A = k1 * cosTheta + k2 + k3 + cosTheta; // C.34
        const double B = -2 * sinTheta; // C.35
        const double C = k1 * cosTheta - k2 + k3 - cosTheta; // C.36

        const double D = std::sqrt(B * B - 4 * A * C);

        const double psi = 2 * std::atan((-B + D) / (2 * A)); // C.39

        return psi;
    }
} // namespace VirtualRobot::four_bar
