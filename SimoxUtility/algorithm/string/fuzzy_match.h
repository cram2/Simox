#pragma once

#ifdef DIFFLIB_FOUND

#include <string>

namespace simox::alg
{

    /**
     * @brief Compare `a` and `b` and return a match ratio between 0 and 1.
     * @param a First string.
     * @param b Second string.
     * @return Match ratio of `a` and `b` in [0, 1].
     */
    double fuzzy_match(const std::string& a, const std::string& b);

} // namespace simox::alg

#endif
