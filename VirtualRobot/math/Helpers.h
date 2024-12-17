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
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

#include <SimoxUtility/math/pose/invert.h>
#include <SimoxUtility/math/pose/pose.h>

#include "MathForwardDefinitions.h"

namespace math
{
    class Helpers
    {
    public:
        static Eigen::Vector3f
        GetOrthonormalVectors(Eigen::Vector3f vec, Eigen::Vector3f& dir1, Eigen::Vector3f& dir2);
        static float ShiftAngle0_2PI(float a);
        static float AngleModPI(float value);
        static void GetIndex(float t, float minT, float maxT, int count, int& i, float& f);
        static float Clamp(float min, float max, float value);
        static int Clampi(int min, int max, int value);
        static float Lerp(float a, float b, float f);
        static Eigen::Vector3f Lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float f);
        static Eigen::Quaternionf
        Lerp(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b, float f);
        static Eigen::Quaternionf
        LerpClamp(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b, float f);
        static float ILerp(float a, float b, float f);
        static float Lerp(float a, float b, int min, int max, int val);
        static float Angle(Eigen::Vector2f v);
        static float
        Angle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& up);
        static int Sign(float x);
        static void AssertNormalized(Eigen::Vector3f vec, float epsilon = 0.05f);
        static std::vector<float> FloatRange(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRangeSymmetric(float start, float end, int steps);
        static std::vector<Eigen::Vector3f>
        VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals);
        static float SmallestAngle(Eigen::Vector3f a, Eigen::Vector3f b);

        static Eigen::Vector3f CwiseMin(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f CwiseMax(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f CwiseDivide(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f CwiseClamp(const Eigen::Vector3f& min,
                                          const Eigen::Vector3f& max,
                                          const Eigen::Vector3f& val);
        static Eigen::Vector3f CwiseClamp(float min, float max, const Eigen::Vector3f& val);
        static Eigen::Vector3f Average(const std::vector<Eigen::Vector3f>& vectors);
        static void Swap(float& a, float& b);

        static Eigen::Vector3f LimitTo(const Eigen::Vector3f& val, float maxNorm);

        // POSE UTILITY
        // These methods have been moved to <SimoxUtility/math/pose.h>, but remain here for
        // backward compatibility.

        /// Get the position block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 1>
        Position(Eigen::MatrixBase<Derived>& pose)
        {
            return simox::math::position(pose);
        }

        /// Get the position block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 1>
        Position(const Eigen::MatrixBase<Derived>& pose)
        {
            return simox::math::position(pose);
        }

        /// Get the orientation block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 3>
        Orientation(Eigen::MatrixBase<Derived>& pose)
        {
            return simox::math::orientation(pose);
        }

        /// Get the orientation block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 3>
        Orientation(const Eigen::MatrixBase<Derived>& pose)
        {
            return simox::math::orientation(pose);
        }

        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename OriDerived>
        static Eigen::Matrix4f
        Pose(const Eigen::MatrixBase<PosDerived>& pos, const Eigen::MatrixBase<OriDerived>& ori)
        {
            return simox::math::pose(pos, ori);
        }

        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename OriDerived>
        static Eigen::Matrix4f
        Pose(const Eigen::MatrixBase<PosDerived>& pos,
             const Eigen::RotationBase<OriDerived, 3>& ori)
        {
            return simox::math::pose(pos, ori);
        }

        /// Build a pose matrix from the given position and identity orientation.
        template <typename Derived,
                  std::enable_if_t<Eigen::MatrixBase<Derived>::RowsAtCompileTime == 3 &&
                                       Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1,
                                   int> = 0>
        static Eigen::Matrix4f
        Pose(const Eigen::MatrixBase<Derived>& position)
        {
            return simox::math::pose(position);
        }

        /// Build a pose matrix from the given orientation and zero position.
        template <typename Derived,
                  std::enable_if_t<Eigen::MatrixBase<Derived>::RowsAtCompileTime == 3 &&
                                       Eigen::MatrixBase<Derived>::ColsAtCompileTime == 3,
                                   int> = 0>
        static Eigen::Matrix4f
        Pose(const Eigen::MatrixBase<Derived>& orientation)
        {
            return simox::math::pose(orientation);
        }

        /// Build a pose matrix from the given orientation and zero position.
        template <typename OriDerived>
        static Eigen::Matrix4f
        Pose(const Eigen::RotationBase<OriDerived, 3>& ori)
        {
            return simox::math::pose(ori);
        }

        static Eigen::Matrix4f CreateTranslationPose(const Eigen::Vector3f& pos);
        static Eigen::Matrix4f CreateRotationPose(const Eigen::Matrix3f& ori);
        static Eigen::Matrix4f
        CreateTranslationRotationTranslationPose(const Eigen::Vector3f& translation1,
                                                 const Eigen::Matrix3f& rotation,
                                                 const Eigen::Vector3f& translation2);

        /// Legacy alias for Pose().
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos,
                                          const Eigen::Quaternionf& ori);
        /// Legacy alias for Pose().
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori);

        static Eigen::Matrix3f CreateOrientation(const Eigen::Vector3f& e1,
                                                 const Eigen::Vector3f& e2,
                                                 const Eigen::Vector3f& e3);

        /// Legacy alias for Position() as getter.
        static Eigen::Vector3f GetPosition(const Eigen::Matrix4f& pose);
        /// Legacy alias for Orientation() as getter.
        static Eigen::Matrix3f GetOrientation(const Eigen::Matrix4f& pose);

        /// Translate the given pose by the given offset.
        static Eigen::Matrix4f TranslatePose(const Eigen::Matrix4f& pose,
                                             const Eigen::Vector3f& offset);
        static Eigen::Matrix4f
        TranslatePose(const Eigen::Matrix4f& pose, float x, float y, float z);

        static Eigen::Matrix4f TranslateAndRotatePose(const Eigen::Matrix4f& pose,
                                                      const Eigen::Vector3f& offset,
                                                      const Eigen::Matrix3f& rotation);

        /// Invert the given pose in-place.
        static void InvertPose(Eigen::Matrix4f& pose);

        /// Return the inverted of the given pose.
        template <typename Derived>
        static Eigen::Matrix4f
        InvertedPose(const Eigen::MatrixBase<Derived>& pose)
        {
            return simox::math::inverted_pose(pose);
        }

        /// Scale the translation/position of the given pose.
        static void ScaleTranslation(Eigen::Matrix4f& pose, float scale);
        /// Get the pose with its translation/position scaled.
        static Eigen::Matrix4f ScaledTranslation(const Eigen::Matrix4f& pose, float scale);

        /// Get a cartesian vector from cylinder coordinates.
        static Eigen::Vector3f CreateVectorFromCylinderCoords(float r, float angle, float z);
        /// Get a cartesian vector from cylinder coordinates.
        static Eigen::Vector3f CartesianFromCylinder(float radius, float angle, float height);
        /// Get a cartesian vector from sphere coordinates.
        static Eigen::Vector3f CartesianFromSphere(float radius, float elevation, float azimuth);

        /// Get a rotation matrix rotating source to target.
        static Eigen::Matrix3f GetRotationMatrix(const Eigen::Vector3f& source,
                                                 const Eigen::Vector3f& target);

        static Eigen::Matrix3f RotateOrientationToFitVector(const Eigen::Matrix3f& ori,
                                                            const Eigen::Vector3f& localSource,
                                                            const Eigen::Vector3f& globalTarget);

        /**
         * @brief Get the tranformation matrix from a source to a target frame.
         *
         * The returned transformation matrix transforms local poses in the
         * source frame to local poses in the target frame.
         *
         * Given poses T_A and T_B of frames A, B relative to some global frame G
         * (i.e. A --T_A--> G, B --T_B--> G), the transformation T_AB (with A --T_AB--> B)
         * is T_AB = (T_B)^-1 * T_A. (Since A --T_A--> G --T_B^-1--> B).
         *
         * @param sourceFramePose The source frame's pose T_A (relative to some global frame).
         * @param targetFramePose The target frame's pose T_B (relative to the same global frame).
         * @return The transformation matrix T_AB from the source frame to the target frame.
         */
        static Eigen::Matrix4f GetTransformFromTo(const Eigen::Matrix4f& sourceFramePose,
                                                  const Eigen::Matrix4f& targetFramePose);

        /// Transform the position by the transform.
        static Eigen::Vector3f TransformPosition(const Eigen::Matrix4f& transform,
                                                 const Eigen::Vector3f& pos);
        /// Transform the direction by the transform.
        static Eigen::Vector3f TransformDirection(const Eigen::Matrix4f& transform,
                                                  const Eigen::Vector3f& dir);
        /// Transform the orientation by the transform.
        static Eigen::Matrix3f TransformOrientation(const Eigen::Matrix4f& transform,
                                                    const Eigen::Matrix3f& ori);

        /// Indicates whether the matrix is orthogonal, i.e. matrix * matrix.transpose = identity.
        template <typename Derived>
        static bool IsMatrixOrthogonal(const Eigen::MatrixBase<Derived>& matrix,
                                       float precision = 1e-6f);

        /// Compute the closest orthogonal matrix to the given matrix.
        /// (Note: All rotation matrices must be orthogonal.)
        static Eigen::Matrix3f Orthogonalize(const Eigen::Matrix3f& matrix);

        /// Orthogonolize the given matrix using Householder QR decomposition.
        static Eigen::Matrix3f OrthogonalizeQR(const Eigen::Matrix3f& matrix);

        /// Orthogonolize the given matrix using Jacobi SVD decomposition.
        static Eigen::Matrix3f OrthogonalizeSVD(const Eigen::Matrix3f& matrix);

        /// Orthogonolize the orientation of the given pose, and sanitize its lower row.
        static Eigen::Matrix4f Orthogonalize(const Eigen::Matrix4f& pose);


        static float
        Distance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float rad2mmFactor);

        static Eigen::VectorXf LimitVectorLength(const Eigen::VectorXf& vec,
                                                 const Eigen::VectorXf& maxLen);


        // Rotation vectors:

        static Eigen::AngleAxisf GetAngleAxisFromTo(const Eigen::Matrix3f& start,
                                                    const Eigen::Matrix3f& target);
        static Eigen::Vector3f GetRotationVector(const Eigen::Matrix3f& start,
                                                 const Eigen::Matrix3f& target);
        static Eigen::Matrix3f RotationVectorToOrientation(const Eigen::Vector3f& rotation);

        // Vector projections:
        static float ScalarProjection(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f VectorProjection(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f VectorRejection(const Eigen::Vector3f& a, const Eigen::Vector3f& b);


        /// Convert a value from radian to degree.
        static float rad2deg(float rad);
        /// Convert a value from degree to radian.
        static float deg2rad(float deg);

        /// Convert an Eigen Matrix or Vector from radian to degree.
        template <typename Derived>
        static Eigen::
            Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
            rad2deg(const Eigen::MatrixBase<Derived>& rad);

        /// Convert an Eigen Matrix or Vector from radian to degree.
        template <typename Derived>
        static Eigen::
            Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
            deg2rad(const Eigen::MatrixBase<Derived>& deg);

        static std::vector<Eigen::Matrix4f>
        CreatePoses(const std::vector<Eigen::Vector3f>& positions,
                    const Eigen::Matrix3f& orientation);

        static std::vector<Eigen::Matrix4f>
        CreatePoses(const Eigen::Vector3f& position,
                    const std::vector<Eigen::Matrix3f>& orientations);

        static std::vector<Eigen::Matrix4f>
        CreatePoses(const std::vector<Eigen::Vector3f>& positions,
                    const std::vector<Eigen::Matrix3f>& orientations);

        // Conversions
        static std::vector<float>
        VectorToStd(const Eigen::VectorXf& vec)
        {
            std::vector<float> res;
            res.resize(static_cast<std::size_t>(vec.size()));
            Eigen::VectorXf::Map(res.data(), vec.size()) = vec;
            return res;
        }

        template <typename T>
        static size_t
        ArgMin(const std::vector<T>& vec)
        {
            if (vec.size() == 0)
                return 0;
            T minVal = vec.at(0);
            size_t minIndex = 0;
            for (size_t i = 1; i < vec.size(); i++)
            {
                T val = vec.at(i);
                if (val < minVal)
                {
                    minVal = val;
                    minIndex = i;
                }
            }
            return minIndex;
        }

        template <typename TVec, typename TSelect>
        static size_t
        ArgMin(const std::vector<TVec>& vec, std::function<TSelect(const TVec&)> selector)
        {
            if (vec.size() == 0)
                return 0;
            TSelect minVal = selector(vec.at(0));
            size_t minIndex = 0;
            for (size_t i = 1; i < vec.size(); i++)
            {
                TSelect val = selector(vec.at(i));
                if (val < minVal)
                {
                    minVal = val;
                    minIndex = i;
                }
            }
            return minIndex;
        }


    private:
    };

    template <typename Derived>
    bool
    Helpers::IsMatrixOrthogonal(const Eigen::MatrixBase<Derived>& matrix, float precision)
    {
        return (matrix * matrix.transpose()).isIdentity(precision);
    }

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
    Helpers::rad2deg(const Eigen::MatrixBase<Derived>& rad)
    {
        return rad * (180.0 / M_PI);
    }

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
    Helpers::deg2rad(const Eigen::MatrixBase<Derived>& deg)
    {
        return deg * (M_PI / 180.0);
    }
} // namespace math

namespace Eigen
{
    template <typename Derived>
    std::ostream&
    operator<<(std::ostream& os, const Eigen::QuaternionBase<Derived>& quat)
    {
        os << "[ " << quat.w() << " | " << quat.x() << " " << quat.y() << "  " << quat.z() << " ]";
        return os;
    }
} // namespace Eigen
