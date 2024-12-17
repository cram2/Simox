#include "assert.h"

#include <experimental/source_location>
#include <sstream>

#include <VirtualRobot/VirtualRobotException.h>

namespace simox::geometric_planning::assert::virtual_robot::detail
{
    void
    check(const bool expressionResult,
          const std::string& expression,
          const std::experimental::source_location& sourceLocation)
    {
        if (not expressionResult)
        {
            std::stringstream stream;
            stream << "Check `" << expression << "` failed in function `"
                   << sourceLocation.function_name() << "` line " << sourceLocation.line() << " at "
                   << sourceLocation.file_name();
            throw VirtualRobot::VirtualRobotException(stream.str());
        }
    }

    void
    checkMessage(bool expressionResult,
                 const std::string& expression,
                 const std::function<std::string()>& messageFtor,
                 const std::experimental::source_location& sourceLocation)
    {
        if (not expressionResult)
        {
            std::stringstream stream;
            stream << "Check `" << expression << "` failed in function `"
                   << sourceLocation.function_name() << "` line " << sourceLocation.line() << " at "
                   << sourceLocation.file_name() << ". " << messageFtor();
            throw VirtualRobot::VirtualRobotException(stream.str());
        }
    }

} // namespace simox::geometric_planning::assert::virtual_robot::detail
