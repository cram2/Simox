
#include "PathProcessor.h"

#include "../CSpace/CSpacePath.h"
#include "../CSpace/CSpaceSampled.h"

namespace Saba
{
    PathProcessor::PathProcessor(CSpacePathPtr path, bool verbose) : path(path), verbose(verbose)
    {
        THROW_VR_EXCEPTION_IF((!path), "NULl path...");
        stopOptimization = false;
    }

    PathProcessor::~PathProcessor() = default;

    CSpacePathPtr
    PathProcessor::getOptimizedPath()
    {
        return optimizedPath;
    }

    void
    PathProcessor::stopExecution()
    {
        stopOptimization = true;
    }
} // namespace Saba
