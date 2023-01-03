
#pragma once

class SoNode;

namespace simox::geometric_planning
{
    class ParametricPath;

    SoNode* visualize(const ParametricPath& path);

    namespace detail
    {
        SoNode* visualizePathLine(const ParametricPath& path);
        SoNode* visualizeCircleSegment(const ParametricPath& path);
    } // namespace detail

} // namespace simox::geometric_planning
