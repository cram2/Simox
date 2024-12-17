#pragma once

#include <VirtualRobot/Robot.h>

#include "BodySanitizer.h"

namespace VirtualRobot::mujoco
{
    class MergedBodyList
    {
    public:
        MergedBodyList();
        MergedBodyList(const std::string& bodyName);

        void addBody(const std::string& bodyName);
        bool containsBody(const std::string& bodyName) const;

        const std::string& getMergedBodyName() const;


    private:
        void updateMergedBodyName();

        std::string mergedBodyName;
        std::vector<std::string> originalBodyNames;
    };

    /**
     * @brief A body sanitizer merging massless bodies to new bodies.
     *
     * Massless bodies are merged along a kinematic chain until a body with
     * mass or a body with multiple children is encountered. The encountered
     * body is the last one included in the merged body.
     *
     * If the last body has multiple children but no mass, a small dummy mass
     * is added.
     */
    class MergingBodySanitizer : public BodySanitizer
    {
    public:
        /// Constructor.
        MergingBodySanitizer(RobotPtr robot);


        /// Set the scaling for lengths (e.g. positions) (to m).
        void setLengthScale(float toMeter);


        /// @see BodySanitizer::sanitize()
        /// @param document Ignored by this class.
        virtual void sanitize(mjcf::Document& document, mjcf::Body root) override;


        // Results.

        const std::vector<MergedBodyList>& getMergedBodyLists() const;

        const std::string& getMergedBodyName(const std::string& originalBodyName);

        MergedBodyList& getMergedBodySetWith(const std::string& bodyName);


    private:
        void sanitizeRecursive(mjcf::Body body);
        void sanitizeLeafBody(mjcf::Body body);

        void mergeBodies(mjcf::Body body, mjcf::Body childBody, Eigen::Matrix4f& accChildPose);

        /// The robot.
        RobotPtr robot;

        /// Scaling factor of lengths (to m).
        float lengthScale = 1.0f;


        // Results.
        std::vector<MergedBodyList> mergedBodyLists;
    };

} // namespace VirtualRobot::mujoco
