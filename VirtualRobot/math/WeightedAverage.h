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

namespace math
{

    struct WeightedFloatAverage
    {
    public:
        void Add(float value, float weight);
        float Average();
        float WeightSum();

    private:
        float sum = 0;
        float weightSum = 0;
    };

    class WeightedVec3Average
    {
    public:
        void Add(Eigen::Vector3f value, float weight);
        Eigen::Vector3f Average();
        float WeightSum();

    private:
        Eigen::Vector3f sum;
        float weightSum = 0;
    };
} // namespace math
