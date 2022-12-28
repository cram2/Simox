#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <experimental/source_location>

#define REQUIRE(a) VR_ASSERT(a)
#define REQUIRE_MESSAGE(a, b) VR_ASSERT_MESSAGE(a, b)

namespace simox::geometric_planning::assert::virtual_robot::detail
{
    void check(bool expressionResult,
               const std::string& expression,
               const std::experimental::source_location& sourceLocation =
                   std::experimental::source_location::current());

} // namespace simox::geometric_planning::assert

#define CHECK(a) ::simox::geometric_planning::assert::virtual_robot::detail::check(a, #a)
