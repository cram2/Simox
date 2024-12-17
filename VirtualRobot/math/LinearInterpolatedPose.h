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
 * @copyright  2019 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

#include "LinearInterpolatedOrientation.h"
#include "MathForwardDefinitions.h"

namespace math
{

    class LinearInterpolatedPose
    {
    public:
        LinearInterpolatedPose(const Eigen::Matrix4f& startPose,
                               const Eigen::Matrix4f& endPose,
                               float startT,
                               float endT,
                               bool clamp);

        Eigen::Matrix4f Get(float t);

    private:
        math::LinearInterpolatedOrientation ori;
        Eigen::Vector3f startPos;
        Eigen::Vector3f endPos;
        float startT;
        float endT;
        bool clamp;
    };
} // namespace math
