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

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace simox
{
    template<class FloatT>
    class OrientedBoxBase
    {
    public:
        template<class T> using vector2_casted   = Eigen::Matrix<T, 2, 1>;
        template<class T> using vector_casted    = Eigen::Matrix<T, 3, 1>;
        template<class T> using transform_casted = Eigen::Matrix<T, 4, 4>;
        template<class T> using rotation_casted  = Eigen::Matrix<T, 3, 3>;

        using float_t = FloatT;
        using vector2_t   = vector2_casted<float_t>;
        using vector_t    = vector_casted<float_t>;
        using transform_t = transform_casted<float_t>;
        using rotation_t  = rotation_casted<float_t>;
    public:
        static constexpr float_t eps = static_cast<float_t>(1e8);
        static constexpr float_t pi  = static_cast<float_t>(M_PI);

    public:
        OrientedBoxBase(const transform_t& t, const vector_t& d) :
            _t{t}, _d{d}
        {}

        OrientedBoxBase() = default;
        OrientedBoxBase(OrientedBoxBase&&) = default;
        OrientedBoxBase(const OrientedBoxBase&) = default;
        OrientedBoxBase& operator=(OrientedBoxBase&&) = default;
        OrientedBoxBase& operator=(const OrientedBoxBase&) = default;

    protected:
        static auto translation(transform_t& t)
        {
            return t.template topRightCorner<3, 1>();
        }
        static auto translation(const transform_t& t)
        {
            return t.template topRightCorner<3, 1>();
        }

        static auto rotation(transform_t& t)
        {
            return t.template topLeftCorner<3, 3>();
        }
        static auto rotation(const transform_t& t)
        {
            return t.template topLeftCorner<3, 3>();
        }

        static transform_t transformation_identity()
        {
            return transform_t::Identity();
        }
        static transform_t transformation(const rotation_t& rot, const vector_t& trans)
        {
            transform_t t = transformation_identity();
            rotation(t) = rot;
            translation(t) = trans;
            return t;
        }

    public:
        const vector_t& dimensions() const
        {
            return _d;
        }
        template<class T>
        vector_casted<T> dimensions() const
        {
            return dimensions().template cast<T>();
        }

        float_t dimension(int i) const
        {
            return _d(i);
        }
        template<class T>
        T dimension(int i) const
        {
            return static_cast<T>(dimension(i));
        }

        float_t dimension_x() const
        {
            return dimension(0);
        }
        template<class T>
        T dimension_x() const
        {
            return dimension<T>(0);
        }

        float_t dimension_y() const
        {
            return dimension(1);
        }
        template<class T>
        T dimension_y() const
        {
            return dimension<T>(1);
        }

        float_t dimension_z() const
        {
            return dimension(2);
        }
        template<class T>
        T dimension_z() const
        {
            return dimension<T>(2);
        }

        const transform_t& transformation() const
        {
            return _t;
        }
        template<class T>
        transform_casted<T> transformation() const
        {
            return transformation().template cast<T>();
        }

        transform_t transformation_centered() const
        {
            return transformation(rotation(), center());
        }
        template<class T>
        transform_casted<T> transformation_centered() const
        {
            return transformation_centered().template cast<T>();
        }

        auto translation() const
        {
            return translation(_t);
        }
        template<class T>
        vector_casted<T> translation() const
        {
            return translation().template cast<T>();
        }

        auto rotation() const
        {
            return rotation(_t);
        }
        template<class T>
        rotation_casted<T> rotation() const
        {
            return rotation().template cast<T>();
        }

        auto axis(int i) const
        {
            return _t.template block<3, 1>(0, i);
        }
        template<class T>
        vector_casted<T> axis(int i) const
        {
            return axis(i).template cast<T>();
        }

        auto axis_x() const
        {
            return axis(0);
        }
        template<class T>
        vector_casted<T> axis_x() const
        {
            return axis_x().template cast<T>();
        }

        auto axis_y() const
        {
            return axis(1);
        }
        template<class T>
        vector_casted<T> axis_y() const
        {
            return axis_y().template cast<T>();
        }

        auto axis_z() const
        {
            return axis(2);
        }
        template<class T>
        vector_casted<T> axis_z() const
        {
            return axis_z().template cast<T>();
        }

        vector_t extend(int i) const
        {
            return axis(i) * dimension(i);
        }
        template<class T>
        vector_casted<T> extend(int i) const
        {
            return extend(i).template cast<T>();
        }

        float_t volume() const
        {
            return _d(0) * _d(1) * _d(2);
        }

        void scale(const vector_t& factors) {
            _d = _d.cwiseProduct(factors);
        }
        void scale(float_t factor)
        {
            _d *= factor;
        }
        void scale_centered(float_t factor)
        {
            translation(_t) += rotation() * _d * (1 - factor) / 2;
            _d *= factor;
        }

        vector_t from_box_frame(const vector_t& p) const
        {
            return translation() + rotation() * p;
        }
        template<class T>
        vector_casted<T> from_box_frame(const vector_t& p) const
        {
            return from_box_frame(p).template cast<T>();
        }

        vector_t to_box_frame(const vector_t& p) const
        {
            return -rotation().transpose() * translation() + rotation().transpose() * p;
        }
        template<class T>
        vector_casted<T> to_box_frame(const vector_t& p) const
        {
            return from_box_frame(p).template cast<T>();
        }

        bool contains(const vector_t& p)
        {
            const vector_t b = to_box_frame(p);
            const auto check_dim = [&](int i)
            {
                return _d(i) < 0 ?
                       (b(i) <= 0 && b(i) >= _d(i)) :
                       (b(i) >= 0 && b(i) <= _d(i));
            };
            return check_dim(0) && check_dim(1) && check_dim(2);
        }

        bool contains_by(const vector_t& p, float_t thresh = 0)
        {
            const vector_t b = to_box_frame(p);
            const auto check_dim = [&](int i)
            {
                return _d(i) < 0 ?
                       (b(i) <= -thresh && b(i) >= _d(i) + thresh) :
                       (b(i) >= +thresh && b(i) <= _d(i) - thresh);
            };
            return check_dim(0) && check_dim(1) && check_dim(2);
        }

        template<class T, class...Other>
        std::vector<Eigen::Matrix<T, 3, 1>, Other...> contained_points(const std::vector<Eigen::Matrix<T, 3, 1>, Other...>& ps)
        {
            std::vector<Eigen::Matrix<T, 3, 1>, Other...> filtered;
            for (const auto& p : ps)
            {
                if (contains(p))
                {
                    filtered.emplace_back(p);
                }
            }
            return filtered;
        }

        vector_t center() const
        {
            return from_box_frame(_d / 2);
        }
        template<class T>
        vector_casted<T> center() const
        {
            return center().template cast<T>();
        }

        vector_t corner_min() const
        {
            return from_box_frame(vector_t::Zero());
        }
        template<class T>
        vector_casted<T> corner_min() const
        {
            return corner_min().template cast<T>();
        }

        vector_t corner_max() const
        {
            return from_box_frame(_d);
        }
        template<class T>
        vector_casted<T> corner_max() const
        {
            return corner_max().template cast<T>();
        }

        vector_t corner(std::uint8_t c) const
        {
            if (c >= 8)
            {
                throw std::invalid_argument{"corner has to be in [0, 7]"};
            }
            const Eigen::Vector3f b
            {
                (c % 2) ? 0 : _d(0),
                ((c / 2) % 2) ? 0 : _d(1),
                ((c / 4) % 2) ? 0 : _d(2)
            };
            return from_box_frame(b);
        }
        template<class T>
        vector_casted<T> corner(std::uint8_t c) const
        {
            return corner(c).template cast<T>();
        }

        std::vector<vector_t> corners() const
        {
            std::vector<Eigen::Vector3f> points;
            points.reserve(8);
            for (std::uint8_t i = 0; i < 8; ++i)
            {
                points.emplace_back(corner(i));
            }
            return points;
        }

    protected:
        transform_t _t{transform_t::Identity()};
        vector_t _d{vector_t::Zero()};
    };
}
