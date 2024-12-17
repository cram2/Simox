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

#include "MathForwardDefinitions.h"

namespace math
{
    class Grid3D
    {
    public:
        Eigen::Vector3f
        P1() const
        {
            return p1;
        }

        Eigen::Vector3f
        P2() const
        {
            return p2;
        }

        int
        StepsX() const
        {
            return stepsX;
        }

        int
        StepsY() const
        {
            return stepsY;
        }

        int
        StepsZ() const
        {
            return stepsZ;
        }

        Eigen::Vector3i
        Steps() const
        {
            return Eigen::Vector3i(stepsX, stepsY, stepsZ);
        }

        Grid3D(Eigen::Vector3f p1, Eigen::Vector3f p2, int stepsX, int stepsY, int stepsZ);
        static Grid3DPtr CreateFromBox(Eigen::Vector3f p1, Eigen::Vector3f p2, float stepLength);
        static Grid3DPtr CreateFromCenterAndSize(const Eigen::Vector3f& center,
                                                 const Eigen::Vector3f& size,
                                                 float stepLength);
        static Grid3DPtr CreateFromCenterAndSteps(const Eigen::Vector3f& center,
                                                  const Eigen::Vector3f& steps,
                                                  float stepLength);

        Eigen::Vector3f Get(int x, int y, int z) const;
        Eigen::Vector3f Get(const Eigen::Vector3i& index) const;
        std::vector<Eigen::Vector3f> AllGridPoints() const;
        Eigen::Vector3i GetFirstIndex() const;
        bool IncrementIndex(Eigen::Vector3i& index) const;
        bool IndexValid(const Eigen::Vector3i& index) const;

    private:
        const Eigen::Vector3f p1;
        const Eigen::Vector3f p2;
        const int stepsX;
        const int stepsY;
        const int stepsZ;
    };
} // namespace math
