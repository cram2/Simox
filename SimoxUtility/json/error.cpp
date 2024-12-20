#include "error.h"

namespace simox::json::error
{

    JsonError::JsonError(const std::string& message) : simox::error::SimoxError(message)
    {
    }

    IOError::IOError(const std::string& msg) : JsonError(msg)
    {
    }

    ParseError::ParseError(const std::string& msg) : JsonError(msg)
    {
    }

    ConversionError::ConversionError(const std::string& msg) : JsonError(msg)
    {
    }

} // namespace simox::json::error
