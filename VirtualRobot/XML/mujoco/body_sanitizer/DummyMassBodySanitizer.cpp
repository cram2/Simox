#include "DummyMassBodySanitizer.h"

namespace VirtualRobot::mujoco
{

    struct DummyMassVisitor : public mjcf::Visitor
    {
        DummyMassVisitor(mjcf::Document& document, const std::string& t = "| ");

        virtual bool visitEnter(const mjcf::AnyElement& element) override;

        const std::string& t;
    };

    DummyMassVisitor::DummyMassVisitor(mjcf::Document& document, const std::string& t) :
        mjcf::Visitor(document), t(t)
    {
    }

    bool
    DummyMassVisitor::visitEnter(const mjcf::AnyElement& element)
    {
        static const bool front = true;

        if (element.is<mjcf::Body>())
        {
            mjcf::Body body = element.as<mjcf::Body>();
            if (!body.hasMass())
            {
                std::cout << t << "Body '" << body.name << ": \tAdd dummy inertial." << std::endl;
                body.addDummyInertial(front);
            }
        }
        return true;
    }

    DummyMassBodySanitizer::DummyMassBodySanitizer()
    {
    }

    void
    DummyMassBodySanitizer::sanitize(mjcf::Document& document, mjcf::Body root)
    {
        DummyMassVisitor visitor(document, t);
        root.accept(visitor);
    }


} // namespace VirtualRobot::mujoco
