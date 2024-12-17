/**
* @package    VirtualRobot
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE VirtualRobot_PQP_optimization

#include <chrono>
#include <random>

#include <Eigen/Geometry>

#include <VirtualRobot/CollisionDetection/PQP/PQP++/OBB_Disjoint.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotTest.h>

BOOST_AUTO_TEST_SUITE(PQP_optimization)

BOOST_AUTO_TEST_CASE(test_obb_disjoint)
{
    std::mt19937 gen;
    std::uniform_real_distribution<float> d{-5, 5};
    std::array<unsigned long, 16> statso{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::array<unsigned long, 16> statsc{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    std::chrono::nanoseconds told{0};
    std::chrono::nanoseconds twloops{0};

    static constexpr auto N = 1000000;

    unsigned long dummy = 0;

    for (std::size_t i = 0; i < N; ++i)
    {
        const Eigen::Matrix3f ori = VirtualRobot::MathTools::rpy2eigen3f(d(gen), d(gen), d(gen));
        PQP::PQP_REAL B[3][3]; //ori B in A

        B[0][0] = ori(0, 0);
        B[0][1] = ori(0, 1);
        B[0][2] = ori(0, 2);

        B[1][0] = ori(1, 0);
        B[1][1] = ori(1, 1);
        B[1][2] = ori(1, 2);

        B[2][0] = ori(2, 0);
        B[2][1] = ori(2, 1);
        B[2][2] = ori(2, 2);

        PQP::PQP_REAL T[3] = {d(gen), d(gen), d(gen)}; // trans of B in A
        PQP::PQP_REAL a[3] = {d(gen), d(gen), d(gen)}; // dim a
        PQP::PQP_REAL b[3] = {d(gen), d(gen), d(gen)}; // dim b

        int wloops = -1;
        int old = -2;

        {
            // force things into cache
            for (std::size_t i = 0; i < 10; ++i)
            {
                dummy += static_cast<unsigned long>(
                    PQP::OBB_Processor::obb_disjoint_with_loops(B, T, a, b));
            }
        }

        if (i % 2)
        {
            {
                const auto start = std::chrono::high_resolution_clock::now();
                wloops = PQP::OBB_Processor::obb_disjoint_with_loops(B, T, a, b);
                const auto end = std::chrono::high_resolution_clock::now();
                twloops += end - start;
            }
            {
                const auto start = std::chrono::high_resolution_clock::now();
                old = PQP::OBB_Processor::obb_disjoint(B, T, a, b);
                const auto end = std::chrono::high_resolution_clock::now();
                told += end - start;
            }
        }
        else
        {
            {
                const auto start = std::chrono::high_resolution_clock::now();
                old = PQP::OBB_Processor::obb_disjoint(B, T, a, b);
                const auto end = std::chrono::high_resolution_clock::now();
                told += end - start;
            }
            {
                const auto start = std::chrono::high_resolution_clock::now();
                wloops = PQP::OBB_Processor::obb_disjoint_with_loops(B, T, a, b);
                const auto end = std::chrono::high_resolution_clock::now();
                twloops += end - start;
            }
        }


        if (i % N / 100 == 0)
        {
            BOOST_CHECK_EQUAL(old, wloops);
        }
        ++statsc.at(wloops);
        ++statso.at(old);
    }
    for (std::size_t i = 0; i < statso.size(); ++i)
    {
        std::cout << i << " -> (o/c) " << statso.at(i) << "\t/\t" << statsc.at(i) << '\n';
    }
    std::cout << "t old        = " << told.count() << '\n';
    std::cout << "t with loops = " << twloops.count() << '\n';
    std::cout << "rel speedup  = " << 1.f * told.count() / twloops.count() << '\n';
}


BOOST_AUTO_TEST_SUITE_END()
