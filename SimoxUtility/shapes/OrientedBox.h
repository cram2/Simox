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
    class OrientedBox : public OrientedBoxBase<FloatT>
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
        OrientedBox(OrientedBox&&) = default;
        OrientedBox(const OrientedBox&) = default;
        OrientedBox& operator=(OrientedBox&&) = default;
        OrientedBox& operator=(const OrientedBox&) = default;

        OrientedBox(const vector_t& corner = {0, 0, 0},
                    const vector_t& extend0 = {1, 0, 0},
                    const vector_t& extend1 = {0, 1, 0},
                    const vector_t& extend2 = {0, 0, 1})
        {
            const float_t len0 = extend0.norm();
            const float_t len1 = extend1.norm();
            const float_t len2 = extend2.norm();
            const vector_t normalized0 = extend0 / len0;
            const vector_t normalized1 = extend1 / len1;
            const vector_t normalized2 = extend2 / len2;

            const float_t dot01 = normalized0.dot(normalized1);
            const float_t dot02 = normalized0.dot(normalized2);
            const float_t dot12 = normalized0.dot(normalized1);

            const float_t angle01 = std::acos(dot01) * 180 / base::pi;
            const float_t angle02 = std::acos(dot02) * 180 / base::pi;
            const float_t angle12 = std::acos(dot12) * 180 / base::pi;

            //checks
            if (std::abs(angle01) > base::eps)
            {
                throw std::invalid_argument{"extend0 and extend1 are not perpendicular! (angle = " +
                                            std::to_string(angle01) + "°)"};
            }
            if (std::abs(angle02) > base::eps)
            {
                throw std::invalid_argument{"extend0 and extend2 are not perpendicular! (angle = " +
                                            std::to_string(angle02) + "°)"};
            }
            if (std::abs(angle12) > base::eps)
            {
                throw std::invalid_argument{"extend1 and extend2 are not perpendicular! (angle = " +
                                            std::to_string(angle12) + "°)"};
            }

            //build transform
            this->_t = transform_t::Identity();
            this->_t.template block<3, 1>(0, 0) = normalized0;
            this->_d(0) = len0;

            const vector_t cross01 = normalized0.cross(normalized1);
            const float_t direction_match = cross01.dot(normalized2);

            if (direction_match > 0)
            {
                this->_t.template block<3, 1>(0, 1) = normalized1;
                this->_t.template block<3, 1>(0, 2) = normalized2;
                this->_d(1) = len1;
                this->_d(2) = len2;
            }
            else
            {
                this->_t.template block<3, 1>(0, 1) = normalized2;
                this->_t.template block<3, 1>(0, 2) = normalized1;
                this->_d(1) = len2;
                this->_d(2) = len1;
            }
            this->_t.template block<3, 1>(0, 3) = corner;
        }

        OrientedBox(const vector_t& center,
                    const rotation_t& orientation,
                    const vector_t& extents) :
            OrientedBox(center - orientation * extents / 2,
                        orientation.col(0) * extents(0),
                        orientation.col(1) * extents(1),
                        orientation.col(2) * extents(2))
        {
        }

        OrientedBox(const transform_t& center_pose, const vector_t& extents) :
            OrientedBox(center_pose.template block<3, 1>(0, 3).eval(),
                        center_pose.template block<3, 3>(0, 0).eval(),
                        extents)
        {
        }

        OrientedBox(const vector_t& center,
                    const Eigen::Quaternion<FloatT>& ori,
                    const vector_t& extents) :
            OrientedBox(center, ori.toRotationMatrix(), extents)
        {
        }

        OrientedBox(const base& b) : base(b)
        {
        }

        template <class T>
        OrientedBox<T>
        cast() const
        {
            return {this->template translation<T>(),
                    this->template axis_x<T>() * this->template dimension<T>(0),
                    this->template axis_y<T>() * this->template dimension<T>(1),
                    this->template axis_z<T>() * this->template dimension<T>(2)};
        }

        OrientedBox
        transformed(const rotation_t& t) const
        {
            return {t * this->translation(),
                    t * this->axis_x() * this->dimension(0),
                    t * this->axis_y() * this->dimension(1),
                    t * this->axis_z() * this->dimension(2)};
        }

        template <class T>
        OrientedBox<T>
        transformed(const rotation_t& t) const
        {
            return transformed(t).template cast<T>();
        }

        OrientedBox
        transformed(const transform_t& t) const
        {
            return {this->rotation(t) * this->translation() + base::translation(t),
                    this->rotation(t) * this->axis_x() * this->dimension(0),
                    this->rotation(t) * this->axis_y() * this->dimension(1),
                    this->rotation(t) * this->axis_z() * this->dimension(2)};
        }

        template <class T>
        OrientedBox<T>
        transformed(const transform_t& t) const
        {
            return transformed(t).template cast<T>();
        }
    };

    using OrientedBoxf = OrientedBox<float>;
    using OrientedBoxd = OrientedBox<double>;

} // namespace simox
