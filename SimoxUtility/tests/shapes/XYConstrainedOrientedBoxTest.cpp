/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / shapes / XYConstrainedOrientedBoxTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/shapes/XYConstrainedOrientedBox.h>

BOOST_AUTO_TEST_CASE(test_XYConstrainedOrientedBox)
{
    static constexpr double eps = 1e-5;

    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> d{1e-4, 10};

    const auto dif_pos = [](const auto& a, const auto& b) { return (a - b).norm(); };
    const auto dif_rot = [](const auto& a, const auto& b)
    { return Eigen::AngleAxisd{a.transpose() * b}.angle(); };

    const auto check =
        [&](const float yaw, const Eigen::Vector3d& corner, const Eigen::Vector3d& dim)
    {
        const auto vol = std::abs(dim(0) * dim(1) * dim(2));
        const Eigen::Matrix3d rot =
            Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
        const Eigen::Vector3d cent = corner + rot * 0.5 * dim;
        std::cout << "#################################################################\nrot\n"
                  << rot << "\ntranl " << corner.transpose() << "\ndim   " << dim.transpose()
                  << "\ncent  " << cent.transpose() << "\nyaw   " << yaw
                  << "\n.................................................................\n";
        const auto subt = [&](auto... vars)
        {
            simox::XYConstrainedOrientedBox<double> b{corner, vars...};
            std::cout << "---- scale 1"
                      << "\nrot\n"
                      << b.rotation() << "\ntranl " << b.translation().transpose() << "\t\t("
                      << corner.transpose() << ')' << "\ndim   " << b.dimensions().transpose()
                      << "\t\t(" << dim.transpose() << ')' << "\ncent  " << b.center().transpose()
                      << "\t\t(" << cent.transpose() << ')' << "\nyaw   " << b.yaw() << "\t\t("
                      << yaw << ")\n";
            BOOST_CHECK_LT(std::abs(b.yaw() - yaw), eps);
            BOOST_CHECK_LT(std::abs(b.volume() - vol), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.translation(), corner), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.dimensions(), dim), eps);
            BOOST_CHECK(b.contains(cent));
            BOOST_CHECK(b.contains_by(corner, -eps));

            const auto sc = d(gen);
            b.scale_centered(sc);
            std::cout << "----scale " << sc << "\nrot\n"
                      << b.rotation() << "\ntranl " << b.translation().transpose() << "\ndim   "
                      << b.dimensions().transpose() << "\ncent  " << b.center().transpose()
                      << "\nyaw   " << b.yaw() << '\n';
            BOOST_CHECK_LT(std::abs(b.yaw() - yaw), eps);
            BOOST_CHECK_LT(std::abs(b.volume() - vol * sc * sc * sc), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.dimensions(), dim * sc), eps);
            BOOST_CHECK(b.contains(cent));

            b.scale_centered(0.0);
            std::cout << "----scale " << 0 << "\nrot\n"
                      << b.rotation() << "\ntranl " << b.translation().transpose() << "\ndim   "
                      << b.dimensions().transpose() << "\ncent  " << b.center().transpose()
                      << "\nyaw   " << b.yaw() << '\n';
            BOOST_CHECK_LT(std::abs(b.yaw() - yaw), eps);
            BOOST_CHECK_LT(std::abs(b.volume()), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.translation(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.dimensions(), Eigen::Vector3d::Zero()), eps);
        };

        const Eigen::Vector2d rx{rot.col(0)(0) * dim(0), rot.col(0)(1) * dim(0)};
        const Eigen::Vector2d ry{rot.col(1)(0) * dim(1), rot.col(1)(1) * dim(1)};

        std::cout << ">>>>>>>>>x" << rx.transpose() << ", y " << ry.transpose() << "\n";
        subt(rx, ry, dim(2));
        std::cout << ">>>>>>>>>y" << ry.transpose() << ", x " << rx.transpose() << "\n";
        subt(ry, rx, dim(2));
        std::cout << ">>>>>>>>>yaw " << yaw << "\n";
        subt(yaw, dim);
    };

    std::uniform_real_distribution<double> y{-M_PI, M_PI};
    for (int i = 0; i < 100; ++i)
    {
        check(y(gen), Eigen::Vector3d::Random() * 100, Eigen::Vector3d::Random().cwiseAbs() * 10);
    }
}
