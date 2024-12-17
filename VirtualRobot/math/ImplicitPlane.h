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

#include "AbstractFunctionR3R1.h"
#include "MathForwardDefinitions.h"

namespace math
{

    class ImplicitPlane : public SimpleAbstractFunctionR3R1
    {
    public:
        // ax + by + cz = d
        ImplicitPlane(float a, float b, float c, float d);

        float
        A()
        {
            return a;
        }

        float
        B()
        {
            return b;
        }

        float
        C()
        {
            return c;
        }

        float
        D()
        {
            return d;
        }

        ImplicitPlane Normalize();
        ImplicitPlane Flipped();
        Eigen::Vector3f GetNormal();
        Eigen::Vector3f GetClosestPoint(Eigen::Vector3f v);
        static ImplicitPlane FromPositionNormal(Eigen::Vector3f pos, Eigen::Vector3f normal);
        static ImplicitPlane FromContact(Contact c);
        float GetSignedDistance(const Eigen::Vector3f& p);

        // https://de.wikipedia.org/wiki/Schnittgerade
        Line Intersect(Plane plane);

        float Get(Eigen::Vector3f pos) override;

    private:
        float a;
        float b;
        float c;
        float d;
    };
} // namespace math
