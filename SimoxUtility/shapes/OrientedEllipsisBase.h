#pragma once

#include <Eigen/Core>

namespace simox
{
    template <class FloatT>
    class OrientedEllipsisBase
    {
    public:
        template <class T>
        using vector2_casted = Eigen::Matrix<T, 2, 1>;
        template <class T>
        using transform3x3_casted = Eigen::Matrix<T, 3, 3>;
        template <class T>
        using rotation2x2_casted = Eigen::Matrix<T, 2, 2>;

        using float_t = FloatT;
        using vector2_t = vector2_casted<float_t>;
        using transform3x3_t = transform3x3_casted<float_t>;
        using rotation2x2_t = rotation2x2_casted<float_t>;

    public:
        static constexpr float_t eps = static_cast<float_t>(1e8);
        static constexpr float_t pi = static_cast<float_t>(M_PI);

    public:
        OrientedEllipsisBase(const transform3x3_t& t, const vector2_t& d) : _t{t}, _d{d}
        {
        }

        OrientedEllipsisBase() = default;
        OrientedEllipsisBase(OrientedEllipsisBase&&) = default;
        OrientedEllipsisBase(const OrientedEllipsisBase&) = default;
        OrientedEllipsisBase& operator=(OrientedEllipsisBase&&) = default;
        OrientedEllipsisBase& operator=(const OrientedEllipsisBase&) = default;

    protected:
        static auto
        translation(transform3x3_t& t)
        {
            return t.template topRightCorner<2, 1>();
        }

        static auto
        translation(const transform3x3_t& t)
        {
            return t.template topRightCorner<2, 1>();
        }

        static auto
        rotation(transform3x3_t& t)
        {
            return t.template topLeftCorner<2, 2>();
        }

        static auto
        rotation(const transform3x3_t& t)
        {
            return t.template topLeftCorner<2, 2>();
        }

        static transform3x3_t
        transformation_identity()
        {
            return transform3x3_t::Identity();
        }

        static transform3x3_t
        transformation(const rotation2x2_t& rot, const vector2_t& trans)
        {
            transform3x3_t t = transformation_identity();
            rotation(t) = rot;
            translation(t) = trans;
            return t;
        }

    public:
        float_t
        radius_x() const
        {
            return _d(0);
        }

        template <class T>
        T
        radius_x() const
        {
            return static_cast<T>(radius_x());
        }

        float_t
        radius_y() const
        {
            return _d(1);
        }

        template <class T>
        T
        radius_y() const
        {
            return static_cast<T>(radius_y());
        }

        const vector2_t&
        transformation() const
        {
            return _t;
        }

        template <class T>
        vector2_casted<T>
        transformation() const
        {
            return transformation().template cast<T>();
        }

        vector2_t
        center() const
        {
            return translation();
        }

        template <class T>
        vector2_casted<T>
        center() const
        {
            return center().template cast<T>();
        }

        auto
        translation() const
        {
            return translation(_t);
        }

        template <class T>
        vector2_casted<T>
        translation() const
        {
            return translation().template cast<T>();
        }

        auto
        rotation() const
        {
            return rotation(_t);
        }

        template <class T>
        rotation2x2_casted<T>
        rotation() const
        {
            return rotation().template cast<T>();
        }

        float_t
        area() const
        {
            return radius_x() * radius_y() * pi;
        }

        void
        scale(float_t factor)
        {
            _d *= factor;
        }

        void
        scale_x(float_t factor)
        {
            _d(0) *= factor;
        }

        void
        scale_y(float_t factor)
        {
            _d(1) *= factor;
        }

    protected:
        transform3x3_t _t{transform3x3_t::Identity()};
        vector2_t _d{vector2_t::Zero()};
    };
} // namespace simox
