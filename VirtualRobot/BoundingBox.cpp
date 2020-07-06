#include "CollisionDetection/CollisionChecker.h"
#include "BoundingBox.h"

#include <iostream>

namespace VirtualRobot
{

    BoundingBox::BoundingBox()
    {
        clear();
    }

    bool BoundingBox::planeGoesThrough(const MathTools::Plane& p)
    {
        bool left = false;
        bool right = false;

        std::vector < Eigen::Vector3f > points = getPoints();

        for (int i = 0; i < 8; i++)
            if (MathTools::onNormalPointingSide(points[i], p))
            {
                left = true;
            }
            else
            {
                right = true;
            }

        if (!left || !right)
        {
            return false;
        }

        return true;
    }


    BoundingBox::BoundingBox(const std::vector< Eigen::Vector3f >& p)
    {
        clear();
        if (p.size() != 0)
        {
            addPoints(p);
        }
    }

    std::vector <Eigen::Vector3f> BoundingBox::getPoints() const
    {
        std::vector < Eigen::Vector3f > points;

        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return points;
        }

        points.push_back(Eigen::Vector3f(min(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), max(2)));

        points.push_back(Eigen::Vector3f(max(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), max(2)));

        return points;
    }

    void BoundingBox::print()
    {
        std::cout << "* Bounding Box\n";
        std::streamsize pr = std::cout.precision(2);
        std::cout << "** min <" << min(0) << "," << min(1) << "," << min(2) << ">\n";
        std::cout << "** max <" << max(0) << "," << max(1) << "," << max(2) << ">\n";
        std::cout.precision(pr);

    }

    void BoundingBox::addPoints(const std::vector < Eigen::Vector3f >& p)
    {
        for (size_t i = 0; i < p.size(); i++)
        {
            addPoint(p[i]);
        }
    }

    void BoundingBox::addPoints(const BoundingBox& bbox)
    {
        std::vector <Eigen::Vector3f> p = bbox.getPoints();
        addPoints(p);
    }

    void BoundingBox::addPoint(const Eigen::Vector3f& p)
    {
        for (int j = 0; j < 3; j++)
        {
            if (std::isnan(min(j)) || p(j) < min(j))
            {
                min(j) = p(j);
            }

            if (std::isnan(max(j)) || p(j) > max(j))
            {
                max(j) = p(j);
            }
        }
    }

    Eigen::Vector3f BoundingBox::getMax() const
    {
        return max;
    }

    Eigen::Vector3f BoundingBox::getMin() const
    {
        return min;
    }


    void BoundingBox::clear()
    {
        min(0) = NAN;
        min(1) = NAN;
        min(2) = NAN;
        max(0) = NAN;
        max(1) = NAN;
        max(2) = NAN;
    }

    void BoundingBox::transform(Eigen::Matrix4f& pose)
    {
        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return;
        }

        Eigen::Vector3f result[8];
        std::vector<Eigen::Vector3f> result3;
        result[0] << min(0), min(1), min(2);
        result[1] << max(0), min(1), min(2);
        result[2] << min(0), max(1), min(2);
        result[3] << max(0), max(1), min(2);
        result[4] << min(0), min(1), max(2);
        result[5] << max(0), min(1), max(2);
        result[6] << min(0), max(1), max(2);
        result[7] << max(0), max(1), max(2);
        Eigen::Matrix4f m;

        for (const auto & i : result)
        {
            m.setIdentity();
            m.block(0, 3, 3, 1) = i;
            m = pose * m;
            result3.push_back(m.block(0, 3, 3, 1));
        }

        // now find min max values
        min << FLT_MAX, FLT_MAX, FLT_MAX;
        max << -FLT_MAX, -FLT_MAX, -FLT_MAX;

        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (result3[i](j) < min(j))
                {
                    min(j) = result3[i](j);
                }

                if (result3[i](j) > max(j))
                {
                    max(j) = result3[i](j);
                }
            }
        }
    }

    void BoundingBox::scale(const Eigen::Vector3f& scaleFactor)
    {
        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return;
        }

        for (int i = 0; i < 3; i++)
        {
            min(i) *= scaleFactor(i);
            max(i) *= scaleFactor(i);
        }
    }

    std::string BoundingBox::toXML(int tabs, bool skipMatrixTag)
    {
            std::stringstream ss;
            std::string t;

            for (int i = 0; i < tabs; i++)
            {
                t += "\t";
            }

            if (!skipMatrixTag)
            {
                ss << t << "<BoundingBox>\n";
            }

            ss << t << "\t<min x='" << min(0) << "' y='" << min(1) << "' z='" << min(2) << "'/>\n";
            ss << t << "\t<max x='" << max(0) << "' y='" << max(1) << "' z='" << max(2) << "'/>\n";

            if (!skipMatrixTag)
            {
                ss << t << "</BoundingBox>\n";
            }

            return ss.str();
    }

    bool BoundingBox::isInside(Eigen::Vector3f point) const
    {
        Eigen::Vector3f p0(min(0), min(1), min(2));
        Eigen::Vector3f p1(min(0), min(1), max(2));
        Eigen::Vector3f p2(min(0), max(1), min(2));
        Eigen::Vector3f p4(max(0), min(1), min(2));

        Eigen::Vector3f v01 = p0 - p1;
        Eigen::Vector3f v02 = p0 - p2;
        Eigen::Vector3f v04 = p0 - p4;

        point(0) = std::isnan(point(0)) ? (max(0) + min(0)) / 2 : point(0);
        point(1) = std::isnan(point(1)) ? (max(1) + min(1)) / 2 : point(1);
        point(2) = std::isnan(point(2)) ? (max(2) + min(2)) / 2 : point(2);

        auto isBetween = [](float x, float a, float b) {
            if (a <= b)
            {
                return a <= x && x <= b;
            }
            else
            {
                return b <= x && x <= a;
            }
        };

        return isBetween(v01.dot(point), v01.dot(p0), v01.dot(p1))
                && isBetween(v02.dot(point), v02.dot(p0), v02.dot(p2))
                && isBetween(v04.dot(point), v04.dot(p0), v04.dot(p4));
    }
}
