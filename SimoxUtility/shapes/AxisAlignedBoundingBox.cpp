#include "AxisAlignedBoundingBox.h"

#include <SimoxUtility/error/SimoxError.h>

namespace simox
{

    AxisAlignedBoundingBox::AxisAlignedBoundingBox()
    {
        set_center({0, 0, 0});
        set_extents({0, 0, 0});
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Eigen::Vector3f& min,
                                                   const Eigen::Vector3f& max)
    {
        this->min() = min;
        this->max() = max;
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Eigen::Vector2f& limits_x,
                                                   const Eigen::Vector2f& limits_y,
                                                   const Eigen::Vector2f& limits_z)
    {
        this->limits_x() = limits_x;
        this->limits_y() = limits_y;
        this->limits_z() = limits_z;
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(float x_min,
                                                   float x_max,
                                                   float y_min,
                                                   float y_max,
                                                   float z_min,
                                                   float z_max)
    {
        this->min() = Eigen::Vector3f(x_min, y_min, z_min);
        this->max() = Eigen::Vector3f(x_max, y_max, z_max);
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Eigen::Matrix32f& limits) : _limits(limits)
    {
    }

    AxisAlignedBoundingBox
    AxisAlignedBoundingBox::from_points(const std::vector<Eigen::Vector3f>& points)
    {
        if (points.empty())
        {
            throwSimoxError("Points must not be empty in `AxisAlignedBoundingBox::from_points().`");
        }

        AxisAlignedBoundingBox aabb(points.front(), points.front());
        for (const auto& p : points)
        {
            aabb.expand_to(p);
        }
        return aabb;
    }

    Eigen::Matrix32f
    AxisAlignedBoundingBox::limits() const
    {
        return _limits;
    }

    Eigen::Matrix32f&
    AxisAlignedBoundingBox::limits()
    {
        return _limits;
    }

    void
    AxisAlignedBoundingBox::set_limits(const Eigen::Matrix32f& value)
    {
        this->_limits = value;
    }

    Eigen::Vector3f
    AxisAlignedBoundingBox::min() const
    {
        return _limits.col(0);
    }

    Eigen::Matrix32f::ColXpr
    AxisAlignedBoundingBox::min()
    {
        return _limits.col(0);
    }

    void
    AxisAlignedBoundingBox::set_min(const Eigen::Vector3f& value)
    {
        min() = value;
    }

    Eigen::Vector3f
    AxisAlignedBoundingBox::max() const
    {
        return _limits.col(1);
    }

    Eigen::Matrix32f::ColXpr
    AxisAlignedBoundingBox::max()
    {
        return _limits.col(1);
    }

    void
    AxisAlignedBoundingBox::set_max(const Eigen::Vector3f& value)
    {
        max() = value;
    }

    Eigen::Vector3f
    AxisAlignedBoundingBox::center() const
    {
        return 0.5 * (min() + max());
    }

    void
    AxisAlignedBoundingBox::set_center(const Eigen::Vector3f& value)
    {
        // Keep current extents, move center.
        const Eigen::Vector3f halfExtents = 0.5 * extents();
        min() = value - halfExtents;
        max() = value + halfExtents;
    }

    Eigen::Vector3f
    AxisAlignedBoundingBox::extents() const
    {
        return max() - min();
    }

    void
    AxisAlignedBoundingBox::set_extents(const Eigen::Vector3f& value)
    {
        // Keep current center, move extents.
        const Eigen::Vector3f& _center = center();
        min() = _center - value / 2;
        max() = _center + value / 2;
    }

    Eigen::Vector2f
    AxisAlignedBoundingBox::limits_x() const
    {
        return _limits.row(0);
    }

    Eigen::Matrix32f::RowXpr
    AxisAlignedBoundingBox::limits_x()
    {
        return _limits.row(0);
    }

    void
    AxisAlignedBoundingBox::set_limits_x(const Eigen::Vector2f& value)
    {
        limits_x() = value;
    }

    Eigen::Vector2f
    AxisAlignedBoundingBox::limits_y() const
    {
        return _limits.row(1);
    }

    Eigen::Matrix32f::RowXpr
    AxisAlignedBoundingBox::limits_y()
    {
        return _limits.row(1);
    }

    void
    AxisAlignedBoundingBox::set_limits_y(const Eigen::Vector2f& value)
    {
        limits_y() = value;
    }

    Eigen::Vector2f
    AxisAlignedBoundingBox::limits_z() const
    {
        return _limits.row(2);
    }

    Eigen::Matrix32f::RowXpr
    AxisAlignedBoundingBox::limits_z()
    {
        return _limits.row(2);
    }

    void
    AxisAlignedBoundingBox::set_limits_z(const Eigen::Vector2f& value)
    {
        limits_z() = value;
    }

    float
    AxisAlignedBoundingBox::min_x() const
    {
        return _limits(0, 0);
    }

    float&
    AxisAlignedBoundingBox::min_x()
    {
        return _limits(0, 0);
    }

    void
    AxisAlignedBoundingBox::set_min_x(const float value)
    {
        max_x() = value;
    }

    float
    AxisAlignedBoundingBox::max_x() const
    {
        return _limits(0, 1);
    }

    float&
    AxisAlignedBoundingBox::max_x()
    {
        return _limits(0, 1);
    }

    void
    AxisAlignedBoundingBox::set_max_x(const float value)
    {
        max_y() = value;
    }

    float
    AxisAlignedBoundingBox::min_y() const
    {
        return _limits(1, 0);
    }

    float&
    AxisAlignedBoundingBox::min_y()
    {
        return _limits(1, 0);
    }

    void
    AxisAlignedBoundingBox::set_min_y(const float value)
    {
        min_y() = value;
    }

    float
    AxisAlignedBoundingBox::max_y() const
    {
        return _limits(1, 1);
    }

    float&
    AxisAlignedBoundingBox::max_y()
    {
        return _limits(1, 1);
    }

    void
    AxisAlignedBoundingBox::set_max_y(const float value)
    {
        max_y() = value;
    }

    float
    AxisAlignedBoundingBox::min_z() const
    {
        return _limits(2, 0);
    }

    float&
    AxisAlignedBoundingBox::min_z()
    {
        return _limits(2, 0);
    }

    void
    AxisAlignedBoundingBox::set_min_z(const float value)
    {
        min_z() = value;
    }

    float
    AxisAlignedBoundingBox::max_z() const
    {
        return _limits(2, 1);
    }

    float&
    AxisAlignedBoundingBox::max_z()
    {
        return _limits(2, 1);
    }

    void
    AxisAlignedBoundingBox::set_max_z(const float value)
    {
        max_z() = value;
    }

    bool
    AxisAlignedBoundingBox::empty(float prec) const
    {
        return extents().isZero(prec);
    }

    float
    AxisAlignedBoundingBox::central_squared_distance(const AxisAlignedBoundingBox& other) const
    {
        return aabb::central_squared_distance(*this, other);
    }

    float
    AxisAlignedBoundingBox::central_distance(const AxisAlignedBoundingBox& other) const
    {
        return aabb::central_distance(*this, other);
    }

    bool
    AxisAlignedBoundingBox::is_colliding(const AxisAlignedBoundingBox& b) const
    {
        return aabb::is_colliding(*this, b);
    }

    float
    aabb::central_distance(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs)
    {
        return (lhs.center() - rhs.center()).norm();
    }

    float
    aabb::central_squared_distance(const AxisAlignedBoundingBox& lhs,
                                   const AxisAlignedBoundingBox& rhs)
    {
        return (lhs.center() - rhs.center()).squaredNorm();
    }

    bool
    aabb::is_colliding(const AxisAlignedBoundingBox& a, const AxisAlignedBoundingBox& b)
    {
        return (a.min_x() <= b.max_x() and a.max_x() >= b.min_x() and a.min_y() <= b.max_y() and
                a.max_y() >= b.min_y() and a.min_z() <= b.max_z() and a.max_z() >= b.min_z());
    }

    bool
    aabb::is_inside(const AxisAlignedBoundingBox& aabb, const Eigen::Vector3f& p)
    {
        return aabb.min_x() <= p.x() and aabb.min_y() <= p.y() and aabb.min_z() <= p.z() and
               p.x() <= aabb.max_x() and p.y() <= aabb.max_y() and p.z() <= aabb.max_z();
    }

    void
    simox::AxisAlignedBoundingBox::expand_to(const Eigen::Vector3f& point)
    {
        min() = min().cwiseMin(point);
        max() = max().cwiseMax(point);
    }

    AxisAlignedBoundingBox
    AxisAlignedBoundingBox::expanded_to(const Eigen::Vector3f& point) const
    {
        return {min().cwiseMin(point), max().cwiseMax(point)};
    }

    void
    AxisAlignedBoundingBox::throwSimoxError(const std::string& msg)
    {
        throw simox::error::SimoxError(msg);
    }

} // namespace simox

std::ostream&
simox::operator<<(std::ostream& os, const AxisAlignedBoundingBox rhs)
{
    static const Eigen::IOFormat iof(3, 0, " ", "", "", "", "[", "]");
    return os << "AABB " << rhs.min().format(iof) << " to " << rhs.max().format(iof);
}
