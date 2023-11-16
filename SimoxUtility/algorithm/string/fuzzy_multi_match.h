#pragma once

#ifdef DIFFLIB_FOUND

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace simox::alg
{

    const std::string*
    fuzzy_multi_match(const std::string& word,
                      const std::map<std::string, std::vector<std::string>>& option_to_words);

    const std::string*
    fuzzy_multi_match(const std::string& word,
                      const std::map<std::string, std::vector<std::string>>& option_to_words,
                      std::ostream& log);

    const std::string*
    fuzzy_multi_match(const std::string& word,
                      const std::map<std::string, std::vector<std::string>>& option_to_words,
                      const std::function<double(const std::string& lhs, const std::string& rhs)>&
                          match_ratio_fn);

    const std::string* fuzzy_multi_match(
        const std::string& word,
        const std::map<std::string, std::vector<std::string>>& option_to_words,
        const std::function<double(const std::string& lhs, const std::string& rhs)>& match_ratio_fn,
        std::ostream& log);


} // namespace simox::alg

#endif
