#pragma once

#ifdef DIFFLIB_FOUND

#include <string>

#include <difflib.h>

namespace simox::alg
{
    /**
     * @brief Compare `a` and `b` and return a match ratio between 0 and 1.
     * @param a First sequence.
     * @param b Second sequence.
     * @return Match ratio of `a` and `b` in [0, 1].
     */
    template <class StringT = std::string>
    double
    fuzzy_match(const StringT& a, const StringT& b)
    {
        difflib::SequenceMatcher<StringT> matcher(a, b);
        return matcher.ratio();
    }
} // namespace simox::alg

#endif
