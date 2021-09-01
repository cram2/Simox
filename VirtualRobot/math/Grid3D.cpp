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

#include "Grid3D.h"
#include "cmath"
#include "Helpers.h"

namespace math
{
    Grid3D::Grid3D(Eigen::Vector3f p1, Eigen::Vector3f p2, int stepsX, int stepsY, int stepsZ)
        : p1(p1), p2(p2), stepsX(stepsX), stepsY(stepsY), stepsZ(stepsZ)
    { }

    Grid3DPtr Grid3D::CreateFromBox(Eigen::Vector3f p1, Eigen::Vector3f p2, float stepLength)
    {
        Eigen::Vector3f steps = (p2 - p1) / stepLength;
        return Grid3DPtr(new Grid3D(p1, p2, std::round(steps.x()), std::round(steps.y()), std::round(steps.z())));
    }

    Grid3DPtr Grid3D::CreateFromCenterAndSize(const Eigen::Vector3f& center, const Eigen::Vector3f& size, float stepLength)
    {
        return CreateFromBox(center - size / 2, center + size / 2, stepLength);
    }

    Grid3DPtr Grid3D::CreateFromCenterAndSteps(const Eigen::Vector3f& center, const Eigen::Vector3f& steps, float stepLength)
    {
        return Grid3DPtr(new Grid3D(center - steps * stepLength / 2, center + steps * stepLength / 2, std::round(steps.x()), std::round(steps.y()), std::round(steps.z())));
    }


    Eigen::Vector3f Grid3D::Get(int x, int y, int z) const
    {
        return Eigen::Vector3f(Helpers::Lerp(p1.x(), p2.x(), 0, stepsX, x),
                               Helpers::Lerp(p1.y(), p2.y(), 0, stepsY, y),
                               Helpers::Lerp(p1.z(), p2.z(), 0, stepsZ, z));
    }

    Eigen::Vector3f Grid3D::Get(const Eigen::Vector3i& index) const
    {
        return Get(index.x(), index.y(), index.z());
    }

    std::vector<Eigen::Vector3f> Grid3D::AllGridPoints() const
    {
        std::vector<Eigen::Vector3f> points;
        for (int x = 0; x <= stepsX; x++)
        {
            for (int y = 0; y <= stepsY; y++)
            {
                for (int z = 0; z <= stepsZ; z++)
                {
                    points.push_back(Get(x, y, z));
                }
            }
        }
        return points;
    }

    Eigen::Vector3i Grid3D::GetFirstIndex() const
    {
        return Eigen::Vector3i::Zero();
    }

    bool Grid3D::IncrementIndex(Eigen::Vector3i& index) const
    {
        Eigen::Vector3i steps = Steps();
        for (int i = 0; i < 3; i++)
        {
            index(i)++;
            if (index(i) <= steps(i))
            {
                return true;
            }
            index(i) = 0;
        }
        return false;
    }

    bool Grid3D::IndexValid(const Eigen::Vector3i& index) const
    {
        Eigen::Vector3i steps = Steps();
        for (int i = 0; i < 3; i++)
        {
            if (index(i) < 0 || index(i) > steps(i))
            {
                return false;
            }
        }
        return true;
    }
}
