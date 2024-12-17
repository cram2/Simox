#include "align_box_orientation.h"

#include <set>
#include <vector>

#include <SimoxUtility/math/pose/check_rotation_matrix.h>
#include <SimoxUtility/math/pose/orthogonalize.h>

static const float ROTATION_PRECISION = 1.0e-3;

Eigen::Matrix3f
simox::math::get_rotation_to_align_box(const Eigen::Matrix3f orientation,
                                       const Eigen::Matrix3f canonicOrientation)
{
    /* [ xx xy xz ]   [  x  ]   [       ]
     * [ yx yy yz ] = [  y  ] * [ x y z ]
     * [ zx zy zz ]   [  z  ]   [       ]
     */
    const Eigen::Matrix3f dots = orientation.transpose() * canonicOrientation;

    Eigen::Matrix3f rotation = rotation.Zero();

    std::set<long> canonicalAxes = {0, 1, 2};
    std::vector<long> actualAxes = {0, 1, 2};

    for (long actual : actualAxes)
    {
        long canon = 0;
        float dot = 0;
        for (long c : canonicalAxes)
        {
            if (std::abs(dots(actual, c)) > std::abs(dot))
            {
                canon = c;
                dot = dots(actual, c);
            }
        }
        // Remove from available axes.
        canonicalAxes.erase(canon);
        // Map axis.
        rotation(actual, canon) = dot > 0 ? 1.f : -1.f;
    };

    check_rotation_matrix(rotation, ROTATION_PRECISION);

    return rotation;
}

Eigen::Matrix3f
simox::math::align_box_orientation(const Eigen::Matrix3f orientation,
                                   const Eigen::Matrix3f canonicOrientation)
{
    const Eigen::Matrix3f rotation = get_rotation_to_align_box(orientation, canonicOrientation);
    const Eigen::Matrix3f canonic = orientation * rotation;
    check_rotation_matrix(canonic, ROTATION_PRECISION);
    return canonic;
}

simox::math::AlignedBox
simox::math::align_box_orientation(const Eigen::Matrix3f orientation,
                                   const Eigen::Vector3f extents,
                                   const Eigen::Matrix3f canonicOrientation)
{
    const Eigen::Matrix3f rotation = get_rotation_to_align_box(orientation, canonicOrientation);

    AlignedBox result;
    result.rotation = rotation;
    result.orientation = orientation * rotation;
    result.extents = (extents.transpose() * rotation).cwiseAbs();

    check_rotation_matrix(result.orientation, ROTATION_PRECISION);

    return result;
}
