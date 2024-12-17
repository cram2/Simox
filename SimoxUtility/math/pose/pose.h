#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SimoxUtility/meta/eigen/enable_if_compile_time_size.h>

namespace simox::math
{

    /**
     * @brief Get the position block from the given pose.
     *
     * You can assign to the result:
     * @code
     * Eigen::Matrix4f pose;
     * math::position(pose) = Eigen::Vector3f(1, 2, 3);
     * @endcode
     */
    template <typename Derived>
    Eigen::Block<Derived, 3, 1>
    position(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 1>(pose.derived(), 0, 3);
    }

    /// Get the position block from the given pose.
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 1>
    position(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 1>(pose.derived(), 0, 3);
    }

    /**
    * @brief Get the orientation block from the given pose.
    *
    * You can assign to the result or modify it in-place:
    * @code
    * Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    * math::orientation(pose) = Eigen::Matrix3f::Identity();
    * math::orientation(pose) = Eigen::Quaternionf::Identity().toRotationMatrix();
    * math::orientation(pose).transposeInPlace();
    * @endcode
    */
    template <typename Derived>
    Eigen::Block<Derived, 3, 3>
    orientation(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 3>(pose.derived(), 0, 0);
    }

    /// Get the orientation block from the given pose.
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 3>
    orientation(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 3>(pose.derived(), 0, 0);
    }

    /// Build a pose matrix from the given position and orientation.
    template <typename PosDerived,
              typename OriDerived,
              typename = std::enable_if_t<
                  std::is_same_v<typename PosDerived::Scalar, typename OriDerived::Scalar>>>
    Eigen::Matrix<typename PosDerived::Scalar, 4, 4>
    pose(const Eigen::MatrixBase<PosDerived>& pos, const Eigen::MatrixBase<OriDerived>& ori)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<PosDerived>, 3, 1);
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<OriDerived>, 3, 3);
        Eigen::Matrix<typename PosDerived::Scalar, 4, 4> pose = pose.Identity();
        position(pose) = pos;
        orientation(pose) = ori;
        return pose;
    }

    /// Build a pose matrix from the given position and orientation.
    template <typename PosDerived,
              typename OriDerived,
              typename = std::enable_if_t<
                  std::is_same_v<typename PosDerived::Scalar, typename OriDerived::Scalar>>>
    Eigen::Matrix<typename PosDerived::Scalar, 4, 4>
    pose(const Eigen::MatrixBase<PosDerived>& pos, const Eigen::RotationBase<OriDerived, 3>& ori)
    {
        return pose(pos, ori.toRotationMatrix());
    }

    /// Build a pose matrix from the given position with identity orientation.
    template <typename Derived, simox::meta::enable_if_vec3<Derived, int> = 0>
    Eigen::Matrix<typename Derived::Scalar, 4, 4>
    pose(const Eigen::MatrixBase<Derived>& position)
    {
        return pose(position, Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity());
    }

    /// Build a pose matrix from the given orientation matrix and zero position.
    template <typename Derived, simox::meta::enable_if_mat3<Derived, int> = 0>
    Eigen::Matrix<typename Derived::Scalar, 4, 4>
    pose(const Eigen::MatrixBase<Derived>& orientation)
    {
        return pose(Eigen::Matrix<typename Derived::Scalar, 3, 1>::Zero(), orientation);
    }

    /// Build a pose matrix from the given orientation and zero position.
    template <typename OriDerived>
    Eigen::Matrix<typename OriDerived::Scalar, 4, 4>
    pose(const Eigen::RotationBase<OriDerived, 3>& ori)
    {
        return pose(Eigen::Matrix<typename OriDerived::Scalar, 3, 1>::Zero(), ori);
    }

    template <class Derived>
    simox::meta::enable_if_mat4<Derived>
    scale_position(Eigen::MatrixBase<Derived>& pose, float factor)
    {
        pose.template topRightCorner<3, 1>() *= factor;
    }

    template <class Derived>
    simox::meta::enable_if_mat4<Derived, Eigen::Matrix<typename Derived::Scalar, 4, 4>>
    scaled_position(const Eigen::MatrixBase<Derived>& pose, float factor)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m = pose;
        scale_position(m, factor);
        return m;
    }
} // namespace simox::math
