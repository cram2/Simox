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

#include "LinearInterpolatedPose.h"

#include "Helpers.h"

namespace math
{
    LinearInterpolatedPose::LinearInterpolatedPose(const Eigen::Matrix4f& startPose,
                                                   const Eigen::Matrix4f& endPose,
                                                   float startT,
                                                   float endT,
                                                   bool clamp) :
        ori(Helpers::GetOrientation(startPose),
            Helpers::GetOrientation(endPose),
            startT,
            endT,
            clamp),
        startPos(Helpers::GetPosition(startPose)),
        endPos(Helpers::GetPosition(endPose)),
        startT(startT),
        endT(endT),
        clamp(clamp)
    {
    }

    Eigen::Matrix4f
    LinearInterpolatedPose::Get(float t)
    {
        float f = Helpers::ILerp(startT, endT, t);
        if (clamp)
        {
            f = Helpers::Clamp(0, 1, f);
        }
        return Helpers::CreatePose(Helpers::Lerp(startPos, endPos, f), ori.Get(t));
    }
} // namespace math
