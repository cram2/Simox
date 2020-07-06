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
* @package    VirtualRobot
* @author     Peter Kaiser
* @author     Matthias Hadlich
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/
#pragma once

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/IK/ConstrainedIK.h"
#include "VirtualRobot/IK/Constraint.h"

namespace nlopt
{
    class opt;
}

namespace VirtualRobot
{
    typedef std::shared_ptr<nlopt::opt> OptimizerPtr;

    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedOptimizationIK : public ConstrainedIK, public std::enable_shared_from_this<ConstrainedOptimizationIK>
    {
    public:
        ConstrainedOptimizationIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, float timeout = 0.5, float globalTolerance = std::numeric_limits<float>::quiet_NaN());

        bool initialize() override;
        bool solve(bool stepwise = false) override;
        bool solveStep() override;

        /**
         * This factor limits the interval around the initial robot configuration where random samplings are placed.
         * A value of 1 (the default) means that samplings can in the whole joint limit interval.
         */
        void setRandomSamplingDisplacementFactor(float displacementFactor);

    protected:
        static double optimizationFunctionWrapper(const std::vector<double> &x, std::vector<double> &gradient, void *data);
        static double optimizationConstraintWrapper(const std::vector<double> &x, std::vector<double> &gradient, void *data);

        double optimizationFunction(const std::vector<double> &x, std::vector<double> &gradient);
        double optimizationConstraint(const std::vector<double> &x, std::vector<double> &gradient, const OptimizationFunctionSetup &setup);

        struct AdditionalOutputData
        {
            struct ConstraintInfo
            {
                std::string constraintName;
                bool success;
                double error;
            };

            std::vector<ConstraintInfo> data;

            std::string toString()
            {
                std::stringstream ss;
                for (ConstraintInfo c : data)
                {
                    ss << "Constraint: " << c.constraintName << " Error: " << c.error << " Success: " << c.success << "\n";
                }
                return ss.str();
            }
        };
        bool hardOptimizationFunction(const std::vector<double> &x, double &error, AdditionalOutputData &data);

    protected:
        RobotNodeSetPtr nodeSet;
        OptimizerPtr optimizer;

        float timeout;
        float globalTolerance;

        std::vector<double> currentX;

        unsigned int numIterations;

        float randomSamplingDisplacementFactor;

        float functionValueTolerance;
        float optimizationValueTolerance;

    };

    typedef std::shared_ptr<ConstrainedOptimizationIK> ConstrainedOptimizationIKPtr;
}


