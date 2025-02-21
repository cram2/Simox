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

#include <eigen3/Eigen/Core>

#include "string_conversion.h"

namespace simox::alg
{
    template <typename T,
              int Rows = Eigen::Dynamic,
              typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
    inline Eigen::Matrix<T, Rows, 1>
    to_eigen(const std::vector<std::string>& vec,
             std::locale locale = DEFAULT_LOCALE,
             bool trimElements = false)
    {
        if (Rows != Eigen::Dynamic && Rows != (int)vec.size())
            throw error::SimoxError("Sizes do not match!");
        Eigen::Matrix<T, Rows, 1> res(vec.size());
        for (unsigned int i = 0; i < vec.size(); i++)
        {
            res(i) = to_<T>(vec[i], locale, trimElements);
        }
        return res;
    }

    template <typename T = float,
              typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
    inline Eigen::Matrix<T, Eigen::Dynamic, 1>
    to_eigen_vec(const std::string& str,
                 const std::string& splitBy = "\t ",
                 bool trimElements = true,
                 bool ignoreEmptyElements = true,
                 std::locale locale = DEFAULT_LOCALE)
    {
        return to_eigen<T>(
            split(str, splitBy, trimElements, ignoreEmptyElements, locale), locale, false);
    }

    template <typename T = float,
              typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
    inline Eigen::Matrix<T, Eigen::Dynamic, 1>
    to_eigen_vec_check_rows(const std::string& str,
                            int expectedRows,
                            const std::string& splitBy = "\t ",
                            bool trimElements = true,
                            bool ignoreEmptyElements = true,
                            std::locale locale = DEFAULT_LOCALE)
    {
        return to_eigen<T>(
            split_check_size(str, expectedRows, splitBy, trimElements, ignoreEmptyElements, locale),
            locale,
            false);
    }

    template <typename T,
              unsigned int Rows,
              typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
    inline Eigen::Matrix<T, Rows, 1>
    to_eigen_vec(const std::string& str,
                 const std::string& splitBy = "\t ",
                 bool trimElements = true,
                 bool ignoreEmptyElements = true,
                 std::locale locale = DEFAULT_LOCALE)
    {
        return to_eigen<T, Rows>(
            split_check_size(str, Rows, splitBy, trimElements, ignoreEmptyElements, locale),
            locale,
            false);
    }

    template <typename T,
              int Rows,
              typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr>
    inline std::string
    to_string(const Eigen::Matrix<T, Rows, 1>& vec,
              const std::string& splitBy = " ",
              std::locale locale = DEFAULT_LOCALE)
    {
        std::ostringstream stream;
        stream.imbue(locale);
        for (unsigned int i = 0; i < vec.rows(); ++i)
        {
            stream << to_string(vec(i));
            if (i + 1 != vec.rows())
                stream << splitBy;
        }
        return stream.str();
    }
} // namespace simox::alg
