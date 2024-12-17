#pragma once

#include <optional>
#include <string>
#include <vector>

#include "json.h"

namespace simox::json
{

    /**
     * @brief Get the value at the first key in `keys` contained in `j`.
     *
     * @param j The JSON object.
     * @param keys The keys.
     * @return The value at the first contained key.
     *
     * @throw std::out_of_range If none of the keys is found in `j`.
     */
    json::const_reference at_any_key(const json& j, const std::vector<std::string>& keys);

    template <class T>
    T
    get_at_any_key(const json& j, const std::vector<std::string>& keys)
    {
        return at_any_key(j, keys).get<T>();
    }

    template <class ValueT, class DefaultT = ValueT>
    void
    get_to_if_exists(const json& j,
                     const std::string& key,
                     ValueT& dest,
                     const DefaultT& default_ = ValueT{})
    {
        if (j.count(key))
        {
            j.at(key).get_to(dest);
        }
        else
        {
            dest = default_;
        }
    }

} // namespace simox::json
