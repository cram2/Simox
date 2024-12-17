#pragma once

#include "error.h"
#include "json.h"

namespace simox::json
{

    /**
     * @brief Read a JSON document from the given file.
     *
     * @param filename The name of the file to read from.
     * @return The JSON document.
     *
     * @throw `simox::json::error::IOError` If IO access fails.
     * @throw `simox::json::error::ParseError` If parsing fails.
     */
    json read(const std::string& filename);

    /**
     * @brief Read a JSON document from the given in-stream.
     *
     * @param is The in-stream.
     * @return The JSON document.
     *
     * @throw `simox::json::error::ParseError` If parsing fails.
     */
    json read(std::istream& is);

    /**
     * @brief Read a JSON document from the given file
     * and convert it to the business-object type `BO`.
     *
     * @param filename The name of the file to read from.
     * @return The business object.
     *
     * @throw `simox::json::error::IOError` If IO access fails.
     * @throw `simox::json::error::ParseError` If parsing fails.
     * @throw `simox::json::error::ConversionError`
     *  If converting the JSON document to the BO fails.
     */
    template <class BO>
    BO
    read(const std::string& filename)
    {
        const json j = simox::json::read(filename);
        try
        {
            return j.get<BO>();
        }
        catch (const json::out_of_range& e) // Add other exception types if they come up.
        {
            throw error::ConversionError(e.what());
        }
    }

    /**
     * @brief Write a JSON document to the given file.
     *
     * @param filename The name of the file to write to.
     * @param j The JSON document.
     * @param indent See nlohmann::json::dump().
     * @param indent_char See nlohmann::json::dump().
     *
     * @throw `simox::json::error::IOError` If IO access fails.
     */
    void write(const std::string& filename,
               const json& j,
               const int indent = -1,
               const char indent_char = ' ');

    /**
     * @brief Write a JSON document to the given out-stream.
     *
     * @param os The out-stream.
     * @param j The JSON document.
     * @param indent The number of indents, if nonnegative (see nlohmann::json::dump()).
     * @param indent_char The character used for indents (see nlohmann::json::dump()).
     */
    void
    write(std::ostream& os, const json& j, const int indent = -1, const char indent_char = ' ');

    /**
     * @brief Write a business object as JSON to the given file.
     *
     * @param filename The name of the file to write to.
     * @param bo The business object.
     * @param indent The number of indents, if nonnegative (see nlohmann::json::dump()).
     * @param indent_char The character used for indents (see nlohmann::json::dump()).
     *
     * @throw `simox::json::error::IOError` If IO access fails.
     * @throw `simox::json::error::ConversionError` If the bo could not access fails.
     */
    template <class BO>
    void
    write(const std::string& filename,
          const BO& bo,
          const int indent = -1,
          const char indent_char = ' ')
    {
        json j;
        try
        {
            j = bo;
        }
        catch (const json::out_of_range& e) // Add other exception types if they come up.
        {
            throw error::ConversionError(e.what());
        }
        simox::json::write(filename, j, indent, indent_char);
    }

} // namespace simox::json

// Legacy names.
namespace nlohmann
{

    json read_json(const std::string& filename);
    json read_json(std::istream& is);

    void write_json(const std::string& filename,
                    const json& j,
                    const int indent = -1,
                    const char indent_char = ' ');
    void write_json(std::ostream& os,
                    const json& j,
                    const int indent = -1,
                    const char indent_char = ' ');

} // namespace nlohmann
