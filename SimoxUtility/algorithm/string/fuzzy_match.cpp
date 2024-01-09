#include "fuzzy_match.h"

#ifdef DIFFLIB_FOUND

#include <SimoxUtility/algorithm/string/fuzzy_match.hpp>

namespace simox
{

    double
    alg::fuzzy_match(const std::string& a, const std::string& b)
    {
        return fuzzy_match<std::string>(a, b);
    }

} // namespace simox

#endif
