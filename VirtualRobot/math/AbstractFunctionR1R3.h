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

#include "../VirtualRobot.h"
#include "SimpleAbstractFunctionR1R3.h"

namespace math
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT AbstractFunctionR1R3 :
        public SimpleAbstractFunctionR1R3
    {
    public:
        AbstractFunctionR1R3();
        virtual Eigen::Vector3f GetDerivative(float t) = 0;
        float FindClosestPoint(Eigen::Vector3f p, float t1, float t2, int segments);
        float MoveLengthOnCurve(float x, float l, int steps);
        float GetLength(float t1, float t2, int steps);
        std::vector<Eigen::Vector3f> Sample(float t1, float t2, int segments);
        std::vector<float> Segments(float t1, float t2, int segments);
    private:
    };
}

