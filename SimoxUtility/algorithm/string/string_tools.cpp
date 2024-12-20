#include "string_tools.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <regex>

#include <boost/algorithm/string.hpp>

#include <SimoxUtility/error/SimoxError.h>

namespace simox
{

    std::string
    alg::to_lower(const std::string& str)
    {
        std::string res = str;
        std::transform(res.begin(), res.end(), res.begin(), tolower);
        return res;
    }

    std::string
    alg::to_upper(const std::string& str)
    {
        std::string res = str;
        std::transform(res.begin(), res.end(), res.begin(), toupper);
        return res;
    }

    std::string
    alg::capitalize_words(const std::string& str)
    {
        std::regex regex("\\s[a-z]");
        std::stringstream result;

        long lastEnd = 0;

        for (std::sregex_iterator it(str.begin(), str.end(), regex); it != std::sregex_iterator();
             ++it)
        {
            std::string match = it->str();

            result << it->prefix();
            result << match.at(0) << static_cast<char>(std::toupper(match.at(1)));

            lastEnd = it->position() + it->length();
        }
        if (lastEnd >= 0 && size_t(lastEnd) < str.size())
        {
            result << str.substr(size_t(lastEnd));
        }

        std::string sresult = result.str();
        sresult[0] = static_cast<char>(std::toupper(sresult[0]));
        return sresult;
    }

    void
    alg::trim(std::string& str, const std::locale& locale)
    {
        boost::trim(str, locale);
    }

    std::string
    alg::trim_copy(const std::string& str, const std::locale& locale)
    {
        return boost::trim_copy(str, locale);
    }

    void
    alg::trim_if(std::string& str, const std::string& trim)
    {
        boost::trim_if(str, boost::is_any_of(trim));
    }

    std::string
    alg::trim_copy_if(const std::string& str, const std::string& trim)
    {
        return boost::trim_copy_if(str, boost::is_any_of(trim));
    }

    std::vector<std::string>
    alg::split(const std::string& str,
               const std::string& splitBy,
               bool trimElements,
               bool removeEmptyElements,
               const std::locale& locale)
    {
        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of(splitBy));

        if (trimElements)
        {
            for (std::string& str : strs)
            {
                simox::alg::trim(str, locale);
            }
        }

        if (removeEmptyElements)
        {
            auto eraseIt = std::remove_if(
                strs.begin(), strs.end(), [](const std::string& s) { return s.empty(); });
            strs.erase(eraseIt, strs.end());
        }

        return strs;
    }

    std::vector<std::string>
    alg::split_check_size(const std::string& str,
                          unsigned int expectedSize,
                          const std::string& splitBy,
                          bool trimElements,
                          bool removeEmptyElements,
                          const std::locale& locale)
    {
        std::vector<std::string> strs =
            split(str, splitBy, trimElements, removeEmptyElements, locale);

        if (strs.size() != expectedSize)
            throw error::SimoxError("String " + str + " contains " + std::to_string(strs.size()) +
                                    " instead of " + std::to_string(expectedSize) +
                                    " values seperated by delimiter " + splitBy);
        return strs;
    }

    std::string
    alg::join(const std::vector<std::string>& vec,
              const std::string& delimiter,
              bool trimElements,
              bool ignoreEmptyElements,
              const std::locale& locale)
    {
        std::ostringstream ss;
        ss.imbue(locale);
        for (size_t index = 0; index < vec.size(); index++)
        {
            if (trimElements)
            {
                std::string trimmed = trim_copy(vec.at(index), locale);
                if (ignoreEmptyElements && trimmed.empty())
                    continue;
                ss << trimmed;
            }
            else
            {
                if (ignoreEmptyElements && vec.at(index).empty())
                    continue;
                ss << vec.at(index);
            }
            if (index < vec.size() - 1)
            {
                ss << delimiter;
            }
        }
        return ss.str();
    }

    std::string
    alg::replace_all(const std::string& input,
                     const std::string& search,
                     const std::string& replace)
    {
        return boost::algorithm::replace_all_copy(input, search, replace);
    }

    std::string
    alg::replace_first(const std::string& input,
                       const std::string& search,
                       const std::string& replace)
    {
        return boost::algorithm::replace_first_copy(input, search, replace);
    }

    std::string
    alg::replace_last(const std::string& input,
                      const std::string& search,
                      const std::string& replace)
    {
        return boost::algorithm::replace_last_copy(input, search, replace);
    }

    bool
    alg::starts_with(const std::string& input, const std::string& search)
    {
        return boost::starts_with(input, search);
    }

    bool
    alg::ends_with(const std::string& input, const std::string& search)
    {
        return boost::ends_with(input, search);
    }

    bool
    alg::contains(const std::string& haystack, const std::string& needle)
    {
        return boost::algorithm::contains(haystack, needle);
    }

    time_t
    alg::to_time_t(const std::string& time, const std::string& time_format)
    {
        std::tm tm = {};
        std::istringstream ss(time);
        ss.imbue(std::locale());
        ss >> std::get_time(&tm, time_format.c_str());
        if (ss.fail())
            throw error::SimoxError("Failed converting string '" + time +
                                    "' to time with time format " + time_format);
        else
            return mktime(&tm);
    }

    std::string
    alg::to_string(time_t t, const std::string& time_format)
    {
        char mbstr[100];
        std::strftime(mbstr, 100, time_format.c_str(), std::localtime(&t));
        return std::string(mbstr);
    }

    unsigned long
    alg::count(const std::string& input, const std::string& search)
    {
        auto start = 0U;
        auto end = input.find(search);
        unsigned long num = 0;
        while (end != std::string::npos)
        {
            num++;
            start = end + search.length();
            end = input.find(search, start);
        }
        return num;
    }

    std::string
    alg::remove_prefix(const std::string& string, const std::string& prefix)
    {
        return simox::alg::starts_with(string, prefix)
                   ? string.substr(prefix.size(), std::string::npos)
                   : string;
    }

    std::string
    alg::remove_suffix(const std::string& string, const std::string& suffix)
    {
        return simox::alg::ends_with(string, suffix)
                   ? string.substr(0, string.size() - suffix.size())
                   : string;
    }


} // namespace simox
