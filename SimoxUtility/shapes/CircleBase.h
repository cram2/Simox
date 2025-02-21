#pragma once

#include <eigen3/Eigen/Core>

namespace simox
{
    template <class FloatT>
    class Circle
    {
    public:
        template <class T>
        using vector2_casted = Eigen::Matrix<T, 2, 1>;

        using float_t = FloatT;
        using vector2_t = vector2_casted<float_t>;

    public:
        static constexpr float_t eps = static_cast<float_t>(1e8);
        static constexpr float_t pi = static_cast<float_t>(M_PI);

    public:
        Circle(const vector2_t& t, const float_t& r) : _t{t}, _r{r}
        {
        }

        Circle() = default;
        Circle(Circle&&) = default;
        Circle(const Circle&) = default;
        Circle& operator=(Circle&&) = default;
        Circle& operator=(const Circle&) = default;

    public:
        float_t
        radius() const
        {
            return _r;
        }

        template <class T>
        T
        radius() const
        {
            return static_cast<T>(radius());
        }

        const vector2_t&
        translation() const
        {
            return _t;
        }

        template <class T>
        vector2_casted<T>
        translation() const
        {
            return translation().template cast<T>();
        }

        vector2_t
        center() const
        {
            return _t;
        }

        template <class T>
        vector2_casted<T>
        center() const
        {
            return center().template cast<T>();
        }

        float_t
        axis(int i) const
        {
            return _t(i);
        }

        template <class T>
        T
        axis(int i) const
        {
            return axis(i).template cast<T>();
        }

        float_t
        axis_x() const
        {
            return axis(0);
        }

        template <class T>
        T
        axis_x() const
        {
            return static_cast<T>(axis_x());
        }

        auto
        axis_y() const
        {
            return axis(1);
        }

        template <class T>
        T
        axis_y() const
        {
            return static_cast<T>(axis_y());
        }

        float_t
        area() const
        {
            return static_cast<float_t>(2.0) * _r * pi;
        }

        void
        scale(float_t factor)
        {
            _r *= factor;
        }

    protected:
        vector2_t _t{vector2_t::Zero()};
        float_t _r{0};
    };
} // namespace simox
