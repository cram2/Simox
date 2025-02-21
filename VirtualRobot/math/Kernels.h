/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

#include <eigen3/Eigen/Core>

#include <VirtualRobot/VirtualRobotImportExport.h>

namespace math
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT KernelWithDerivatives
    {
    public:
        virtual float
        Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const = 0;
        virtual float Kernel_dx(float x, float y, float z, float r, float R) const = 0;
        virtual float Kernel_ddx(float x, float y, float z, float r, float R) const = 0;
        virtual float Kernel_dxdy(float x, float y, float z, float r, float R) const = 0;

        float
        Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int i, int j) const;
        float Kernel_di(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int i) const;
        float Kernel_dj(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int j) const;
        float Kernel_didj(const Eigen::Vector3f& p1,
                          const Eigen::Vector3f& p2,
                          float R,
                          int i,
                          int j) const;
        void swap(float& x, float& y, float& z, int index) const;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT GaussianKernel : public KernelWithDerivatives
    {
    public:
        GaussianKernel(float lengthScale);
        float Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const override;
        float Kernel_dx(float x, float y, float z, float r, float R) const override;
        float Kernel_ddx(float x, float y, float z, float r, float R) const override;
        float Kernel_dxdy(float x, float y, float z, float r, float R) const override;

    private:
        float lengthScale;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT WilliamsPlusKernel : public KernelWithDerivatives
    {
        float Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const override;
        float Kernel_dx(float x, float y, float z, float r, float R) const override;
        float Kernel_ddx(float x, float y, float z, float r, float R) const override;
        float Kernel_dxdy(float x, float y, float z, float r, float R) const override;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT WilliamsMinusKernel : public KernelWithDerivatives
    {
        float Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const override;
        float Kernel_dx(float x, float y, float z, float r, float R) const override;
        float Kernel_ddx(float x, float y, float z, float r, float R) const override;
        float Kernel_dxdy(float x, float y, float z, float r, float R) const override;
    };

} // namespace math
