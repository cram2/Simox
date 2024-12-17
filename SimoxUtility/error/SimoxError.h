#pragma once

#include <stdexcept>
#include <string>

namespace simox::error
{
    /**
     * @brief The base class of all errors thrown by `SimoxUtility`.
     */
    class SimoxError : public std::runtime_error
    {
    public:
        SimoxError(const std::string& message);
    };
} // namespace simox::error
