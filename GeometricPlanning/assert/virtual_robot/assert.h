#pragma once

#include <experimental/source_location>
#include <sstream>

#include <VirtualRobot/VirtualRobot.h>
#include "VirtualRobot/Assert.h"

#define REQUIRE(a) VR_ASSERT(a)
#define REQUIRE_MESSAGE(a, b) VR_ASSERT_MESSAGE(a, b)

namespace simox::geometric_planning::assert::virtual_robot::detail
{
    void check(bool expressionResult,
               const std::string& expression,
               const std::experimental::source_location& sourceLocation =
                   std::experimental::source_location::current());

    /**
     * @brief Helper function for lazy string/stream concatenation of var args
     * 
     */
    template <typename... Args>
    auto
    messageFn(Args... args)
    {
        return [=]() -> std::string
        {
            std::stringstream stream;

            // fold expression: https://en.cppreference.com/w/cpp/language/fold
            ([&] { stream << args; }(), ...);

            return stream.str();
        };
    }

    void checkMessage(bool expressionResult,
                      const std::string& expression,
                      const std::function<std::string()>& messageFtor,
                      const std::experimental::source_location& sourceLocation =
                          std::experimental::source_location::current());


} // namespace simox::geometric_planning::assert::virtual_robot::detail

#define CHECK(a) ::simox::geometric_planning::assert::virtual_robot::detail::check(a, #a)
#define CHECK_MESSAGE(a, ...)                                                                      \
    ::simox::geometric_planning::assert::virtual_robot::detail::checkMessage(                      \
        a, #a, ::simox::geometric_planning::assert::virtual_robot::detail::messageFn(__VA_ARGS__))
