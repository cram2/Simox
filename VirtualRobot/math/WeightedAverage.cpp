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

#include "WeightedAverage.h"

namespace math
{
    void math::WeightedFloatAverage::Add(float value, float weight)
    {
        sum += value * weight;
        weightSum += weight;
    }

    float WeightedFloatAverage::Average()
    {
        return sum / weightSum;
    }

    float WeightedFloatAverage::WeightSum()
    {
        return weightSum;
    }
    void math::WeightedVec3Average::Add(Eigen::Vector3f value, float weight)
    {
        sum += value * weight;
        weightSum += weight;
    }

    Eigen::Vector3f WeightedVec3Average::Average()
    {
        return sum / weightSum;
    }

    float WeightedVec3Average::WeightSum()
    {
        return weightSum;
    }
}
