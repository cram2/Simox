#include "util.h"

#include <sstream>

namespace simox
{
    json::json::const_reference
    json::at_any_key(const json& j, const std::vector<std::string>& keys)
    {
        for (const std::string& key : keys)
        {
            if (j.count(key))
            {
                return j.at(key);
            }
        }

        std::stringstream ss;
        ss << "None of these keys found in JSON document: \n";
        for (const auto& k : keys)
        {
            ss << "- '" << k << "'\n";
        }
        throw std::out_of_range(ss.str());
    }
} // namespace simox
