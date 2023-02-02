#include <chrono>
#include <iostream>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/rescale.h>
#include <SimoxUtility/math/statistics/measures.h>

#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Nodes/HemisphereJoint/Maths.h>


using VirtualRobot::RuntimeEnvironment;


int main(int argc, char* argv[])
{
    const Eigen::IOFormat iof(5, 0, " ", "\n", "    [", "]", "", "");

    RuntimeEnvironment::setCaption("Compute hemisphere joint mathematics");

    RuntimeEnvironment::considerFlag(
                "verbose", "Enable verbose output.");
    RuntimeEnvironment::considerFlag(
                "debug", "Branch to debug code.");

    RuntimeEnvironment::processCommandLine(argc, argv);

    if (RuntimeEnvironment::hasHelpFlag()
            // || !RuntimeEnvironment::hasValue("input")
            //|| !RuntimeEnvironment::hasValue("output"))
            )
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    const bool verbose = RuntimeEnvironment::hasFlag("verbose");
    const bool debug = RuntimeEnvironment::hasFlag("debug");

    const double lever = 1, theta0 = simox::math::deg_to_rad(25.);
    std::cout << "(lever, theta0) = (" << lever << ", " << theta0 << ") " << std::endl;

    if (not debug)
    {
        std::vector<double> a1s, a2s;
        int num = 100;
        double aMin = -0.7, aMax=0.7;
        for (int i = 0; i < num; ++i)
        {
            double value = simox::math::rescale(double(i), double(0), double(num), aMin, aMax);
            a1s.push_back(value);
            a2s.push_back(value);
        }


        using time_point = std::chrono::system_clock::time_point;

        std::vector<double> durationsUs;
        durationsUs.reserve(a1s.size() * a2s.size());

        for (double a1 : a1s)
        {
            for (double a2 : a2s)
            {
                const time_point start = std::chrono::system_clock::now();

                VirtualRobot::hemisphere::Maths joint;
                joint.computeFkOfPosition(a1, a2);

                const Eigen::Vector3d pos = joint.getEndEffectorTranslation();
                const Eigen::Matrix3d ori = joint.getEndEffectorRotation();
                const Eigen::Matrix<double, 6, 2> jacobian = joint.getJacobian();

                const time_point end = std::chrono::system_clock::now();
                using duration = std::chrono::nanoseconds;
                durationsUs.push_back(std::chrono::duration_cast<duration>(end - start).count() / 1000.f);

                if (verbose)
                {
                    std::cout << "(a1, a2) = (" << a1 << ", " << a2 << ")"
                              << "\n ->"
                              << "\n  pos = \n" << pos.transpose().format(iof)
                              << "\n  ori = \n" << ori.format(iof)
                              << "\n  jac = \n" << jacobian.format(iof)
                              << std::endl;
                }
            }
        }

        double mean = simox::math::mean(durationsUs);
        double stddev = simox::math::stddev(durationsUs, mean);
        double min = simox::math::min(durationsUs);
        double max = simox::math::max(durationsUs);

        const std::string unit = " us";
        std::cout << "Durations:"
                  << " " << mean << " +- " << stddev << unit
                  << ", range: [" << min << unit << " to " << max << unit <<"]"
                  << std::endl;
    }
    else
    {
        double offset = std::asin(theta0);

        double a1 = 0, a2 = 0;
        a1 += offset;
        a2 += offset;

        VirtualRobot::hemisphere::Maths joint;
        joint.computeFkOfPosition(a1, a2);

        const Eigen::Vector3d pos = joint.getEndEffectorTranslation();
        std::cout << "\n  pos = \n" << pos.transpose().format(iof)
                  << std::endl;

    }

    return 0;
}
