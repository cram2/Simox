#include "fuzzy_multi_match.h"

#include "fuzzy_multi_match.hpp"

#ifdef DIFFLIB_FOUND

namespace simox
{
    const std::string*
    alg::fuzzy_multi_match(
        const std::string& word,
        const std::map<std::string, std::vector<std::string>>& option_to_words,
        const std::function<double(const std::string&, const std::string&)>& match_ratio_fn)
    {
        return fuzzy_multi_match<std::string, std::string, double>(
            word, option_to_words, match_ratio_fn);
    }

    const std::string*
    alg::fuzzy_multi_match(
        const std::string& word,
        const std::map<std::string, std::vector<std::string>>& option_to_words,
        const std::function<double(const std::string&, const std::string&)>& match_ratio_fn,
        std::ostream& log)
    {
        return fuzzy_multi_match<std::string, std::string, double>(
            word, option_to_words, match_ratio_fn, log);
    }

    const std::string*
    alg::fuzzy_multi_match(const std::string& word,
                           const std::map<std::string, std::vector<std::string>>& option_to_words)
    {
        return fuzzy_multi_match<std::string, std::string>(word, option_to_words);
    }

    const std::string*
    alg::fuzzy_multi_match(const std::string& word,
                           const std::map<std::string, std::vector<std::string>>& option_to_words,
                           std::ostream& log)
    {
        return fuzzy_multi_match<std::string, std::string>(word, option_to_words, log);
    }

} // namespace simox

#endif
