#include "SimoxError.h"

namespace simox::error
{
    SimoxError::SimoxError(const std::string& message) : std::runtime_error(message)
    {
    }
} // namespace simox::error
