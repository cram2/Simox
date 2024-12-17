#include "interpolate.h"

namespace simox::math
{


    Eigen::Affine3f
    interpolatePose(const Eigen::Affine3f& posePre, const Eigen::Affine3f& poseNext, float t)
    {

        Eigen::Affine3f pose = Eigen::Affine3f::Identity();

        pose.translation() = (1 - t) * posePre.translation() + t * poseNext.translation();

        Eigen::Quaternionf rotPrev(posePre.linear().matrix());
        Eigen::Quaternionf rotNext(poseNext.linear().matrix());

        Eigen::Quaternionf rotNew = rotPrev.slerp(t, rotNext);

        pose.linear() = rotNew.toRotationMatrix();

        return pose;
    }

    Eigen::Isometry3f
    interpolatePose(const Eigen::Isometry3f& posePre, const Eigen::Isometry3f& poseNext, float t)
    {

        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();

        pose.translation() = (1 - t) * posePre.translation() + t * poseNext.translation();

        Eigen::Quaternionf rotPrev(posePre.linear().matrix());
        Eigen::Quaternionf rotNext(poseNext.linear().matrix());

        Eigen::Quaternionf rotNew = rotPrev.slerp(t, rotNext);

        pose.linear() = rotNew.toRotationMatrix();

        return pose;
    }

} // namespace simox::math
