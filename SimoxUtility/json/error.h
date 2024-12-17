#pragma once

#include <SimoxUtility/error/SimoxError.h>

namespace simox::json::error
{

    /**
     * @brief An exception thrown by the `simox::json` namespace.
     */
    struct JsonError : public simox::error::SimoxError
    {
        JsonError(const std::string& message);
    };

    /**
     * @brief Indicates that IO access to a file failed.
     */
    struct IOError : public JsonError
    {
        IOError(const std::string& msg);
    };

    /**
     * @brief Indicates that a JSON file could not be parsed.
     */
    struct ParseError : public JsonError
    {
        ParseError(const std::string& msg);
    };

    /**
     * @brief Indicates that a JSON document could not be converted to a
     * business object.
     */
    struct ConversionError : public JsonError
    {
        ConversionError(const std::string& msg);
    };

} // namespace simox::json::error
