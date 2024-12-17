/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Andre Meixner (andre dot meixner at kit dot edu)
 * @copyright  2020 Andre Meixner
 *             GNU Lesser General Public License
 */

#pragma once

#include <cstring>
#include <locale>
#include <sstream>
#include <vector>

namespace simox::alg
{

    static std::locale DEFAULT_LOCALE = std::locale::classic();

    std::string to_lower(const std::string& str);

    std::string to_upper(const std::string& str);

    /**
     * @brief Capitalize all words in `str`.
     *
     * More precisely, this makes all lower-case letters occuring after
     * a whitespace upper-case
     */
    std::string capitalize_words(const std::string& str);


    void trim(std::string& str, const std::locale& locale = DEFAULT_LOCALE);

    std::string trim_copy(const std::string& str, const std::locale& locale = DEFAULT_LOCALE);

    void trim_if(std::string& str, const std::string& trim = "\t ");

    std::string trim_copy_if(const std::string& str, const std::string& trim = "\t ");


    std::vector<std::string> split(const std::string& str,
                                   const std::string& splitBy = "\t ",
                                   bool trimElements = true,
                                   bool removeEmptyElements = true,
                                   const std::locale& locale = DEFAULT_LOCALE);


    /// @param expectedSize throws SimoxError if split not matches expectedSize
    std::vector<std::string> split_check_size(const std::string& str,
                                              unsigned int expectedSize,
                                              const std::string& splitBy = "\t ",
                                              bool trimElements = true,
                                              bool removeEmptyElements = true,
                                              const std::locale& locale = DEFAULT_LOCALE);

    std::string join(const std::vector<std::string>& vec,
                     const std::string& delimiter = " ",
                     bool trimElements = false,
                     bool ignoreEmptyElements = false,
                     const std::locale& locale = DEFAULT_LOCALE);


    std::string
    replace_first(std::string const& input, std::string const& search, std::string const& replace);

    std::string
    replace_last(std::string const& input, std::string const& search, std::string const& replace);

    std::string
    replace_all(std::string const& input, std::string const& search, std::string const& replace);


    bool starts_with(std::string const& input, std::string const& search);
    bool ends_with(std::string const& input, std::string const& search);

    bool contains(const std::string& haystack, const std::string& needle);

    inline bool
    contains(const std::string& haystack, const char* needle)
    {
        return contains(haystack, std::string(needle));
    }

    inline bool
    contains(const std::string& string, const char character)
    {
        return string.find(character) != std::string::npos;
    }

    unsigned long count(const std::string& input, const std::string& search);

    inline unsigned long
    count(const std::string& input, const char* search)
    {
        return count(input, std::string(search));
    }

    inline unsigned long
    count(const std::string& input, const char search)
    {
        return count(input, std::to_string(search));
    }

    /**
     * @brief If `string` begins with `prefix`, return `string` without the
     * leading `prefix`.
     */
    std::string remove_prefix(const std::string& string, const std::string& prefix);

    /**
     * @brief If `string` ends with `suffix`, return `string` without the
     * ending `suffix`.
     */
    std::string remove_suffix(const std::string& string, const std::string& suffix);

    template <typename IterT>
    std::vector<std::string>
    multi_to_string(IterT begin, IterT end)
    {
        std::vector<std::string> result;
        for (auto it = begin; it != end; ++it)
        {
            std::stringstream ss;
            ss << *it;
            result.push_back(ss.str());
            ++begin;
        }
        return result;
    }

    template <typename T>
    std::vector<std::string>
    multi_to_string(const std::vector<T>& vector)
    {
        return multi_to_string(vector.begin(), vector.end());
    }

    time_t to_time_t(const std::string& time, const std::string& time_format);

    std::string to_string(time_t t, const std::string& time_format);
} // namespace simox::alg
