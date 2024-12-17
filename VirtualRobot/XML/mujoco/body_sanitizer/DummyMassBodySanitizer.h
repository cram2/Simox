#pragma once

#include "BodySanitizer.h"

namespace VirtualRobot::mujoco
{

    /**
     * @brief A body sanitizer adding a small dummy mass to massless bodies.
     */
    class DummyMassBodySanitizer : public BodySanitizer
    {
    public:
        /// Constructor.
        DummyMassBodySanitizer();


        /// @see BodySanitizer::sanitize()
        virtual void sanitize(mjcf::Document& document, mjcf::Body root) override;
    };

} // namespace VirtualRobot::mujoco
