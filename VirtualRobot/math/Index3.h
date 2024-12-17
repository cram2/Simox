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

    struct Index3
    {
    private:
        int x;
        int y;
        int z;

    public:
        int
        X()
        {
            return x;
        }

        int
        Y()
        {
            return y;
        }

        int
        Z()
        {
            return z;
        }

        void
        SetX(int value)
        {
            x = value;
        }

        void
        SetY(int value)
        {
            y = value;
        }

        void
        SetZ(int value)
        {
            z = value;
        }

        Index3(int x, int y, int z);

        std::string ToString();
    };

    /*static Index3 operator -(Index3 a);
static Index3 operator +(Index3 a, Index3 b);
static Index3 operator -(Index3 a, Index3 b);
static Index3 operator *(Index3 a, int f);
static Index3 operator *(int f, Index3 a);
static Index3 operator /(Index3 a, int f);*/

} // namespace math
