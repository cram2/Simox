#include "io.h"

#include <filesystem>
#include <fstream>
#include <iostream>

#include "error.h"


namespace fs = std::filesystem;

namespace simox
{
    json::json
    json::read(const std::string& filename)
    {
        std::ifstream ifs;
        // Allow throwing std::ios_base::failure.
        ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try
        {
            ifs.open(filename);
            return simox::json::read(ifs);
        }
        catch (const std::ios_base::failure& e)
        {
            // Add a more useful message.
            std::string msg = "Failed to read file \"" + filename + "\": ";
            if (not fs::exists(filename))
            {
                msg += "File \"" + filename + "\" does not exist.";
            }
            else
            {
                msg += std::string(e.what());
            }
            throw simox::json::error::IOError(msg);
        }
    }

    json::json
    json::read(std::istream& is)
    {
        json j;
        try
        {
            is >> j;
        }
        catch (const json::parse_error& e)
        {
            throw error::ParseError(e.what());
        }
        return j;
    }

    void
    json::write(const std::string& filename,
                const json& j,
                const int indent,
                const char indent_char)
    {
        std::ofstream ofs;
        // Allow throwing std::ios_base::failure.
        ofs.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try
        {
            ofs.open(filename);
            simox::json::write(ofs, j, indent, indent_char);
        }
        catch (const std::ios_base::failure& e)
        {
            // Add a more useful message.
            const std::string msg = "Failed to write file \"" + filename + "\": ";
            throw error::IOError(msg + std::string(e.what()));
        }
    }

    void
    json::write(std::ostream& os, const json& j, const int indent, const char indent_char)
    {
        os << j.dump(indent, indent_char);
    }

} // namespace simox

nlohmann::json
nlohmann::read_json(const std::string& filename)
{
    return simox::json::read(filename);
}

nlohmann::json
nlohmann::read_json(std::istream& is)
{
    return simox::json::read(is);
}

void
nlohmann::write_json(const std::string& filename,
                     const json& j,
                     const int indent,
                     const char indent_char)
{
    simox::json::write(filename, j, indent, indent_char);
}

void
nlohmann::write_json(std::ostream& os, const json& j, const int indent, const char indent_char)
{
    simox::json::write(os, j, indent, indent_char);
}
