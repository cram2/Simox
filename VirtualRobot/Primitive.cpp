#include "Primitive.h"
#include "MathTools.h"

namespace VirtualRobot
{
    namespace Primitive
    {

        std::string Primitive::getTransformString(int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<Transform>\n";
            result << MathTools::getTransformXMLString(transform, tabs + 2);
            result << pre << "\t</Transform>\n";
            return result.str();
        }

        std::string Primitive::getXMLString(const std::string& type, const std::string& params, int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<" << type << " " << params << ">\n";
            result << getTransformString(tabs + 1);
            result << pre << "\t</" << type << ">\n";
            return result.str();
        }

        //derivate functions
        std::string Box::toXMLString(int tabs)
        {
            std::stringstream format;
            format << "width=\"" << width
                   << "\" height=\"" << height
                   << "\" depth=\"" << depth << "\"";
            return getXMLString("Box", format.str(), tabs);
        }

        std::string Sphere::toXMLString(int tabs)
        {
            std::stringstream format;
            format << "radius=\"" << radius << "\"";
            return getXMLString("Sphere", format.str(), tabs);
        }

        std::string Cylinder::toXMLString(int tabs)
        {
            std::stringstream format;
            format << "radius=\"" << radius
                   << "\" height=\"" << height << "\"";
            return getXMLString("Cylinder", format.str(), tabs);
        }

        std::string Capsule::toXMLString(int tabs)
        {
            std::stringstream format;
            format << "radius=\"" << radius
                   << "\" height=\"" << height << "\"";
            return getXMLString("Capsule", format.str(), tabs);
        }

    } //namespace Primitive
} //namespace VirtualRobot
