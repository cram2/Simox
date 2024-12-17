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
 * @author     Raphael Grimm (raphael dot grimm at kit dot edu)
 * @copyright  2019 Raphael Grimm
 *             GNU Lesser General Public License
 */

#pragma once

#include "OrientedBoxBase.h"

namespace simox
{
    template <class FloatT>
    class XYConstrainedOrientedBox : public OrientedBoxBase<FloatT>
    {
        using base = OrientedBoxBase<FloatT>;

    public:
        template <class T>
        using vector2_casted = typename base::template vector2_casted<T>;
        template <class T>
        using vector_casted = typename base::template vector_casted<T>;
        template <class T>
        using transform_casted = typename base::template transform_casted<T>;
        template <class T>
        using rotation_casted = typename base::template rotation_casted<T>;

        using float_t = FloatT;
        using vector2_t = typename base::vector2_t;
        using vector_t = typename base::vector_t;
        using transform_t = typename base::transform_t;
        using rotation_t = typename base::rotation_t;

    public:
        XYConstrainedOrientedBox(XYConstrainedOrientedBox&&) = default;
        XYConstrainedOrientedBox(const XYConstrainedOrientedBox&) = default;
        XYConstrainedOrientedBox& operator=(XYConstrainedOrientedBox&&) = default;
        XYConstrainedOrientedBox& operator=(const XYConstrainedOrientedBox&) = default;

        XYConstrainedOrientedBox(const vector_t& corner = {0, 0, 0},
                                 const float_t yaw = 0,
                                 const vector_t& dimensions = {0, 0, 0}) :
            base(base::transformation(
                     Eigen::AngleAxis<float_t>(yaw, vector_t::UnitZ()).toRotationMatrix(),
                     corner),
                 dimensions),
            _yaw{yaw}
        {
        }

    private:
        static std::tuple<vector_t, float_t, vector_t>
        calc_params(const vector_t& corner,
                    const vector2_t& extend0,
                    const vector2_t& extend1,
                    const float_t height)
        {
            const float_t len0 = extend0.norm();
            const float_t len1 = extend1.norm();

            const vector2_t normalized2d0 = extend0 / len0;
            const vector2_t normalized2d1 = extend1 / len1;

            const float_t dot01 = normalized2d0.dot(normalized2d1);
            const float_t angle01 = std::acos(dot01) * 180 / base::pi;

            //checks
            if (std::abs(angle01) > base::eps)
            {
                throw std::invalid_argument{"extend0 and extend1 are not perpendicular! (angle = " +
                                            std::to_string(angle01) + "°)"};
            }

            //make sure the system is right handed + calculate yaw
            const vector_t normalized0{normalized2d0(0), normalized2d0(1), 0};
            const vector_t normalized1{normalized2d1(0), normalized2d1(1), 0};

            const vector_t cross01 = normalized0.cross(normalized1);
            if (cross01(2) >= 0)
            {
                // x=0, y=1
                const float_t yaw = -std::atan2(normalized1(0), normalized1(1));
                return {corner, yaw, vector_t{len0, len1, height}};
            }

            // x=1, y=0
            const float_t yaw = -std::atan2(normalized0(0), normalized0(1));
            return {corner, yaw, vector_t{len1, len0, height}};
        }

        XYConstrainedOrientedBox(std::tuple<vector_t, float_t, vector_t> params) :
            XYConstrainedOrientedBox(std::get<0>(params), std::get<1>(params), std::get<2>(params))
        {
        }

    public:
        XYConstrainedOrientedBox(const vector_t& corner,
                                 const vector2_t& extend0,
                                 const vector2_t& extend1,
                                 const float_t height) :
            XYConstrainedOrientedBox(calc_params(corner, extend0, extend1, height))
        {
        }

        float_t
        yaw() const
        {
            return _yaw;
        }

        template <class T>
        T
        yaw() const
        {
            return _yaw;
        }

        template <class T>
        XYConstrainedOrientedBox<T>
        cast() const
        {
            return {this->template translation<T>(), yaw<T>(), this->template dimensions<T>()};
        }

    private:
        float_t _yaw;
    };
} // namespace simox
