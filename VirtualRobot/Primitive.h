#pragma once

#include <VirtualRobot/VirtualRobot.h>

#include <Eigen/Core>


namespace VirtualRobot
{
    namespace Primitive
    {

        class VIRTUAL_ROBOT_IMPORT_EXPORT Primitive
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            static const int TYPE = 0;
            const int type;
            Eigen::Matrix4f transform;

            virtual std::string toXMLString(int tabs) = 0;

            virtual std::unique_ptr<Primitive> clone() const = 0;

        protected:
            Primitive(int type) : type(type), transform(Eigen::Matrix4f::Identity()) {}
            std::string getTransformString(int tabs = 0);
            std::string getXMLString(const std::string& type, const std::string& params, int tabs = 0);
        private:
            Primitive() : type(TYPE) {}
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Box : public Primitive
        {
        public:
            static const int TYPE = 1;
            Box() : Primitive(TYPE) {}
            Box(float width, float height, float depth) : Primitive(TYPE), width(width), height(height), depth(depth) {}
            float width;
            float height;
            float depth;
            std::string toXMLString(int tabs = 0) override;

            std::unique_ptr<Primitive> clone() const final
            {
                return std::make_unique<Box>(width, height, depth);
            }
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Sphere : public Primitive
        {
        public:
            static const int TYPE = 2;
            Sphere() : Primitive(TYPE) {}
            Sphere(float radius) : Primitive(TYPE), radius(radius) {}
            float radius;
            std::string toXMLString(int tabs = 0) override;

            std::unique_ptr<Primitive> clone() const final
            {
                return std::make_unique<Sphere>(radius);
            }
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Cylinder : public Primitive
        {
        public:
            static const int TYPE = 3;
            Cylinder() : Primitive(TYPE) {}
            Cylinder(float radius, float height) : Primitive(TYPE), radius(radius), height(height) {}
            float radius;
            float height;
            std::string toXMLString(int tabs = 0) override;

            std::unique_ptr<Primitive> clone() const final
            {
                return std::make_unique<Cylinder>(radius, height);
            }
        };

        /**
         * @brief The Capsule class. The capsule is extended along the y-axis.
         */
        class VIRTUAL_ROBOT_IMPORT_EXPORT Capsule : public Primitive
        {
        public:
            static const int TYPE = 4;
            Capsule() : Primitive(TYPE) {}
            Capsule(float radius, float height) : Primitive(TYPE), radius(radius), height(height) {}
            float radius;
            float height;
            std::string toXMLString(int tabs = 0) override;

            std::unique_ptr<Primitive> clone() const final
            {
                return std::make_unique<Capsule>(radius, height);
            }
        };

        typedef std::shared_ptr<Primitive> PrimitivePtr;

    } //namespace Primitive
} //namespace VirtualRobot

