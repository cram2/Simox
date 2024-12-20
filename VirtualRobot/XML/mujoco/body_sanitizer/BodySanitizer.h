#pragma once

#include <VirtualRobot/MJCF/Document.h>

namespace VirtualRobot::mujoco
{

    /**
     * @brief Sanitizer for MJCF bodies without mass.
     *
     * MuJoCo does not allow non-static bodies to have 0 mass. However, this
     * is the case when the robot model contains elements representing joints,
     * which are not associated with some geometry.
     *
     * A body sanitizer edits massless bodies to fix this issue.
     */
    class BodySanitizer
    {
    public:
        /// Constructor.
        BodySanitizer();

        /// Virtual destructor.
        virtual ~BodySanitizer();


        /**
         * @brief Sanitize the given root body and its direct and indirect children.
         * @param document The MJCF document.
         * @param root The root body to sanitize.
         */
        virtual void sanitize(mjcf::Document& document, mjcf::Body root) = 0;


    protected:
        /// "Tab" string for logging.
        static const std::string t;
    };

} // namespace VirtualRobot::mujoco
