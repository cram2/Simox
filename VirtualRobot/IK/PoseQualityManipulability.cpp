#include "PoseQualityManipulability.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../IK/DifferentialIK.h"
#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"

using namespace std;

namespace VirtualRobot
{


    PoseQualityManipulability::PoseQualityManipulability(VirtualRobot::RobotNodeSetPtr rns,
                                                         ManipulabilityIndexType i) :
        PoseQualityMeasurement(rns),
        manipulabilityType(i),
        penJointLimits(false),
        convertMMtoM(true)
    {
        name = getTypeName();
        jacobian.reset(new VirtualRobot::DifferentialIK(rns, rns->getTCP()));
        jacobian->convertModelScalingtoM(convertMMtoM);
        penalizeRotationFactor = 0.15f; // translation<->rotation factors
    }

    PoseQualityManipulability::~PoseQualityManipulability() = default;

    Eigen::MatrixXf
    PoseQualityManipulability::getSingularVectorCartesian()
    {
        if (verbose)
        {
            std::cout << "*** PoseQualityManipulability::getSingularVectorCartesian()\n";
        }

        Eigen::MatrixXf jac = jacobian->getJacobianMatrix(rns->getTCP());
        //cout << "JAC\n:" << jac << std::endl;
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;
        //cout << "scaled JAC\n:" << jac << std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf U = svd.matrixU();
        Eigen::VectorXf sv = svd.singularValues();

        if (verbose)
        {
            std::cout << "U:\n" << U << std::endl;
        }

        if (sv.rows() == 0)
        {
            return U;
        }

        if (verbose)
        {
            std::cout << "sv:\n" << sv << std::endl;
        }

        float maxEV = sv(0);

        if (verbose)
        {
            std::cout << "maxEV:" << maxEV << std::endl;
        }

        /*for (int i=0;i<sv.rows();i++)
        {
            Eigen::MatrixXf Bl = U.block(0,i,U.rows(),1);
            Bl *= sv(i) / maxEV;
            U.block(0,i,U.rows(),1) = Bl;
        }*/
        if (verbose)
        {
            std::cout << "result:\n" << U << std::endl;
        }

        return U;
    }

    float
    PoseQualityManipulability::getManipulability(ManipulabilityIndexType i)
    {
        return getManipulability(i, -1);
    }

    float
    PoseQualityManipulability::getManipulability(const Eigen::VectorXf& direction,
                                                 int /*considerFirstSV*/)
    {
        VR_ASSERT(direction.rows() == 3 || direction.rows() == 6);
        Eigen::VectorXf d(6);

        if (direction.rows() == 6)
        {
            d = direction;
        }
        else
        {
            d.setZero();
            d.segment(0, 3) = direction;
        }

        float result = 0;

        // jacobian (in global coord system!)
        VirtualRobot::DifferentialIKPtr jacobianGlobal(new VirtualRobot::DifferentialIK(rns));
        jacobianGlobal->convertModelScalingtoM(convertMMtoM);

        Eigen::MatrixXf jac = jacobianGlobal->getJacobianMatrix(rns->getTCP());
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;

        // compute gradient
        Eigen::VectorXf gradient = (jac.transpose() * d).transpose();

        result = gradient.norm();


        if (penJointLimits)
        {
            result *= getJointLimitPenalizationFactor();
        }

        return result;
    }

    float
    PoseQualityManipulability::getManipulability(ManipulabilityIndexType i, int considerFirstSV)
    {
        Eigen::VectorXf sv = getSingularValues();

        if (considerFirstSV <= 0 || considerFirstSV > sv.rows())
        {
            considerFirstSV = sv.rows();
        }

        float result = 0.0f;

        switch (i)
        {
            case eMultiplySV:
            {
                if (sv.rows() >= 1)
                {
                    result = sv(0);

                    for (int j = 1; j < considerFirstSV; j++)
                    {
                        result *= sv(j);
                    }
                }

                break;
            }

            case eMinMaxRatio:
            {
                if (sv.rows() >= 2)
                {
                    float maxSV = sv(0);
                    float minSV = sv(considerFirstSV - 1);

                    if (maxSV != 0)
                    {
                        result = minSV / maxSV;
                    }
                }

                break;
            }

            default:
                VR_ERROR << "Manipulability type not implemented..." << std::endl;
        }

        if (penJointLimits)
        {
            result *= getJointLimitPenalizationFactor();
        }

        return result;
    }

    float
    PoseQualityManipulability::getPoseQuality()
    {
        return getManipulability(manipulabilityType);
    }

    float
    PoseQualityManipulability::getPoseQuality(const Eigen::VectorXf& direction)
    {
        return getManipulability(direction);
    }

    Eigen::VectorXf
    PoseQualityManipulability::getSingularValues()
    {
        Eigen::MatrixXf jac = jacobian->getJacobianMatrix(rns->getTCP());
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        return svd.singularValues();
    }

    void
    PoseQualityManipulability::penalizeJointLimits(bool enable, float k /*= 50.0f*/)
    {
        penJointLimits = enable;
        penJointLimits_k = k;
    }

    float
    PoseQualityManipulability::getJointLimitPenalizationFactor()
    {
        VR_ASSERT(rns);
        float p = 1.0f;

        for (unsigned int i = 0; i < rns->getSize(); i++)
        {
            RobotNodePtr r = rns->getNode(i);

            if (r->isLimitless())
            {
                continue;
            }

            VR_ASSERT(r);
            float d = r->getJointLimitHi() - r->getJointLimitLo();
            d = d * d;
            float a = (r->getJointValue() - r->getJointLimitLo()) *
                      (r->getJointLimitHi() - r->getJointValue());

            if (d != 0)
            {
                p *= a / d;
            }
        }

        float result = 1.0f - exp(-penJointLimits_k * p);
        if (verbose)
            std::cout << "Pen factor:" << result << std::endl;
        return result;
    }

    std::string
    PoseQualityManipulability::getTypeName()
    {
        return std::string("PoseQualityManipulability");
    }

    bool
    PoseQualityManipulability::consideringJointLimits()
    {
        return penJointLimits;
    }

    PoseQualityMeasurementPtr
    PoseQualityManipulability::clone(RobotPtr newRobot)
    {
        VR_ASSERT(newRobot);
        VR_ASSERT(newRobot->getRobotNodeSet(rns->getName()));
        VR_ASSERT(newRobot->getRobotNodeSet(rns->getName())->getSize() == rns->getSize());

        PoseQualityManipulabilityPtr m(new PoseQualityManipulability(
            newRobot->getRobotNodeSet(rns->getName()), this->manipulabilityType));
        m->penalizeJointLimits(this->penJointLimits, this->penJointLimits_k);
        return m;
    }


} // namespace VirtualRobot
