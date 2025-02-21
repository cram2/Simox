#pragma once

#include <eigen3/Eigen/Core>

namespace Eigen
{
    using Matrix32f = Eigen::Matrix<float, 3, 2>;
}

namespace simox
{
    /**
     * @brief An axis-aligned (bounding-)box, defined by its limits in
     * x-, y- and z- direction.
     */
    struct AxisAlignedBoundingBox
    {
        /// Construct an empty AABB centered at (0, 0, 0) with extents (0, 0, 0).
        AxisAlignedBoundingBox();

        /// Construct an AABB with minimal and maximal values.
        AxisAlignedBoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
        /// Construct an AABB with axis-wise limits.
        AxisAlignedBoundingBox(const Eigen::Vector2f& limits_x,
                               const Eigen::Vector2f& limits_y,
                               const Eigen::Vector2f& limits_z);
        /// Construct an AABB with given limits.
        AxisAlignedBoundingBox(float x_min,
                               float x_max,
                               float y_min,
                               float y_max,
                               float z_min,
                               float z_max);
        /// Construct an AABB with limits in a 3x2 matrix.
        AxisAlignedBoundingBox(const Eigen::Matrix32f& limits);

        /**
         * @brief Compute the AABB of the given points.
         * @throws `simox::error::SimoxError` If `points` is empty.
         */
        static AxisAlignedBoundingBox from_points(const std::vector<Eigen::Vector3f>& points);
        /**
         * @brief Compute the AABB of the given points.
         * `PointT` must have x, y, z member variables.
         * @throws `simox::error::SimoxError` If `points` is empty.
         */
        template <class PointT>
        static AxisAlignedBoundingBox from_points(const std::vector<PointT>& points);


        Eigen::Matrix32f limits() const;
        Eigen::Matrix32f& limits();
        void set_limits(const Eigen::Matrix32f& value);

        Eigen::Vector3f min() const;
        Eigen::Matrix32f::ColXpr min();
        void set_min(const Eigen::Vector3f& value);

        Eigen::Vector3f max() const;
        Eigen::Matrix32f::ColXpr max();
        void set_max(const Eigen::Vector3f& value);

        Eigen::Vector3f center() const;
        void set_center(const Eigen::Vector3f& value);
        Eigen::Vector3f extents() const;
        void set_extents(const Eigen::Vector3f& value);

        Eigen::Vector2f limits_x() const;
        Eigen::Matrix32f::RowXpr limits_x();
        void set_limits_x(const Eigen::Vector2f& value);

        Eigen::Vector2f limits_y() const;
        Eigen::Matrix32f::RowXpr limits_y();
        void set_limits_y(const Eigen::Vector2f& value);

        Eigen::Vector2f limits_z() const;
        Eigen::Matrix32f::RowXpr limits_z();
        void set_limits_z(const Eigen::Vector2f& value);

        float min_x() const;
        float& min_x();
        void set_min_x(float value);
        float max_x() const;
        float& max_x();
        void set_max_x(float value);

        float min_y() const;
        float& min_y();
        void set_min_y(float value);
        float max_y() const;
        float& max_y();
        void set_max_y(float value);

        float min_z() const;
        float& min_z();
        void set_min_z(float value);
        float max_z() const;
        float& max_z();
        void set_max_z(float value);


        /// Indicate whether this AABB is empty (i.e. has zero extents).
        bool empty(float prec = Eigen::NumTraits<float>::dummy_precision()) const;

        /// Return the distance between center of `*this` and `other`'s center.
        float central_distance(const AxisAlignedBoundingBox& other) const;
        /// Return the squared distance between `*this` center and `other`'s center.
        float central_squared_distance(const AxisAlignedBoundingBox& other) const;

        /// Checks whether `*this` is colliding (i.e. overlapping) with `other`.
        bool is_colliding(const AxisAlignedBoundingBox& other) const;


        /// Indicates whether `point` is inside `*this`.
        template <class PointT>
        bool is_inside(const PointT& p) const;

        /**
         * @brief Expand `*this` (in-place) to include `point`.
         * Note that `*this` should be suitable initialized beforehand.
         * @see from_points()
         */
        template <class PointT>
        void
        expand_to(const PointT& point)
        {
            expand_to(Eigen::Vector3f(point.x, point.y, point.z));
        }

        void expand_to(const Eigen::Vector3f& point);

        /**
         * @brief Return this AABB expanded to include `point`.
         * Note that `*this` should be suitable initialized beforehand.
         * @see from_points()
         */
        template <class PointT>
        void
        expanded_to(const PointT& point) const
        {
            expanded_to(Eigen::Vector3f(point.x, point.y, point.z));
        }

        AxisAlignedBoundingBox expanded_to(const Eigen::Vector3f& point) const;


    private:
        /**
         * @brief Throw a simox::error::SimoxError() with the given message.
         * This avoids exposing the include in the header.
         */
        [[noreturn]] static void throwSimoxError(const std::string& msg);


    private:
        /**
         * The limits structured as follows:
         *
         * @code
         *    min    max
         * [ min_x  max_x ]  <- x
         * [ min_y  max_y ]  <- y
         * [ min_z  max_z ]  <- z
         * @endcode
         */
        Eigen::Matrix32f _limits = Eigen::Matrix32f::Zero();
    };

    std::ostream& operator<<(std::ostream& os, const AxisAlignedBoundingBox rhs);

    namespace aabb
    {
        template <class PointT>
        AxisAlignedBoundingBox
        from_points(const std::vector<PointT>& points)
        {
            return AxisAlignedBoundingBox::from_points<PointT>(points);
        }

        inline AxisAlignedBoundingBox
        from_points(const std::vector<Eigen::Vector3f>& points)
        {
            return AxisAlignedBoundingBox::from_points(points);
        }

        /// Return the distance between center of `lhs`'s and `rhs`'s centers.
        float central_distance(const AxisAlignedBoundingBox& lhs,
                               const AxisAlignedBoundingBox& rhs);

        /// Return the squared distance between `lhs`'s center and `rhs`'s centers.
        float central_squared_distance(const AxisAlignedBoundingBox& lhs,
                                       const AxisAlignedBoundingBox& rhs);

        /// Checks whether `lhs` is colliding (i.e. overlapping) with `rhs`.
        bool is_colliding(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs);

        /// Indicates whether `point` is inside `aabb`.
        bool is_inside(const AxisAlignedBoundingBox& aabb, const Eigen::Vector3f& point);

        /// Indicates whether `point` is inside `aabb`.
        /// `PointT` must have members variables x, y, z.
        template <class PointT>
        bool
        is_inside(const AxisAlignedBoundingBox& aabb, const PointT& p)
        {
            return aabb.min_x() <= p.x and aabb.min_y() <= p.y and aabb.min_z() <= p.z and
                   p.x <= aabb.max_x() and p.y <= aabb.max_y() and p.z <= aabb.max_z();
        }
    } // namespace aabb

    template <class PointT>
    AxisAlignedBoundingBox
    AxisAlignedBoundingBox::from_points(const std::vector<PointT>& points)
    {
        if (points.empty())
        {
            throwSimoxError("Points must not be empty in `AxisAlignedBoundingBox::from_points()`.");
        }

        const PointT& front = points.front();
        AxisAlignedBoundingBox aabb(front.x, front.x, front.y, front.y, front.z, front.z);
        for (const auto& p : points)
        {
            aabb.expand_to(p);
        }
        return aabb;
    }

    template <class PointT>
    bool
    AxisAlignedBoundingBox::is_inside(const PointT& p) const
    {
        return aabb::is_inside(*this, p);
    }

} // namespace simox
