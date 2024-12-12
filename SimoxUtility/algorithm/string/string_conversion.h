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

#include <type_traits>
#include <iomanip>

#include "SimoxUtility/error/SimoxError.h"
#include "string_tools.h"

namespace simox::alg {
    namespace help {
        template <typename T>
        struct type{};

        template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
        inline T to_(const std::string& s, type<T>, std::locale locale = DEFAULT_LOCALE, bool trim = true)
        {
            char c;
            std::stringstream ss;
            ss.imbue(locale);
            if (trim)
                ss << alg::trim_copy(s);
            else
                ss << s;
            T value;
            ss >> value;
            if (ss.fail() || ss.get(c)) {
                throw error::SimoxError("Cannot convert string " + s + " to " + typeid(T).name());
                ss.clear();
            }
            return value;
        }

        template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
        inline T to_(const std::string& s, type<bool>, std::locale /*locale*/, bool trim = true)
        {
            std::string help = to_lower(s);
            if (trim) alg::trim(help);
            if (help == "true" || help == "1") return true;
            else if (help == "false" || help == "0") return false;
            else throw error::SimoxError("Cannot convert string " + s + " to boolean.");
        }
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline T to_(const std::string& s, std::locale locale = DEFAULT_LOCALE, bool trim = true)
    {
        return help::to_<T>(s, help::type<T>{}, locale, trim);
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline std::vector<T> to_vec(const std::vector<std::string>& vec, std::locale locale = DEFAULT_LOCALE, bool trimElements = false)
    {
        std::vector<T> res;
        for (const std::string& str : vec)
        {
            res.push_back(to_<T>(str, locale, trimElements));
        }
        return res;
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline std::vector<T> to_vec(const std::string& str, const std::string& splitBy = "\t ", bool trimElements = true,
                                 bool ignoreEmptyElements = true, std::locale locale = DEFAULT_LOCALE)
    {
        return to_vec<T>(split(str, splitBy, trimElements, ignoreEmptyElements, locale), locale, false);
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline std::vector<T> to_vec_check_size(const std::string& str, unsigned int expectedSize, const std::string& splitBy = "\t ",
                                       bool trimElements = true, bool ignoreEmptyElements = true, std::locale locale = DEFAULT_LOCALE)
    {
        return to_vec<T>(split_check_size(str, expectedSize, splitBy, trimElements, ignoreEmptyElements, locale), locale, false);
    }



    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline std::string to_string(T x, const std::locale& locale = DEFAULT_LOCALE)
    {
        std::stringstream ss;
        ss.imbue(locale);
        ss << x;
        return ss.str();
    }

    template<> inline std::string to_string<double>(double d,  const std::locale& locale)
    {
        std::stringstream ss;
        ss.imbue(locale);
        ss << std::setprecision(12) << d;
        return ss.str();
    }

    template<> inline std::string to_string<bool>(bool x, const std::locale& /*locale*/)
    {
        return x ? "true" : "false";
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    inline std::string to_string(const std::vector<T>& vec, const std::string& splitBy = " ", std::locale locale = DEFAULT_LOCALE)
    {
        std::stringstream stream;
         stream.imbue(locale);
        for(unsigned int i = 0; i < vec.size(); ++i) {
            stream << to_string(vec.at(i));
            if (i + 1 != vec.size()) stream << splitBy;
        }
        return stream.str();
    }

    template<typename T, typename std::enable_if<!std::is_fundamental<T>::value>::type* = nullptr>
    inline std::string to_string(const std::vector<T>& vec, const std::string& splitBy = " ", std::locale locale = DEFAULT_LOCALE)
    {
        std::stringstream stream;
        stream.imbue(locale);
        for(unsigned int i = 0; i < vec.size(); ++i) {
            stream << vec.at(i);
            if (i + 1 != vec.size()) stream << splitBy;
        }
        return stream.str();
    }
}
