#pragma once

#ifdef DIFFLIB_FOUND

#include <iomanip>

#include <SimoxUtility/algorithm/string/fuzzy_match.hpp>

namespace simox::alg::detail
{
    template <class OptionT, class StringT = std::string, class RankT = double>
    struct RatedOption
    {
        const StringT* word;
        RankT ratio;
        const OptionT* option;

        static bool
        compare(const RatedOption& lhs, const RatedOption& rhs)
        {
            return lhs.ratio < rhs.ratio;
        }

        static bool
        compare_reverse(const RatedOption& lhs, const RatedOption& rhs)
        {
            return lhs.ratio > rhs.ratio;
        }
    };

    template <class OptionT, class StringT = std::string, class RankT = double>
    std::vector<RatedOption<OptionT, StringT, RankT>>
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RankT(const StringT& ls, const StringT& rhs)>& matchRatioFn)
    {
        std::vector<RatedOption<OptionT, StringT, RankT>> rated_options;

        for (const auto& [option, option_words] : option_to_words)
        {
            for (const auto& option_word : option_words)
            {
                RankT ratio = matchRatioFn(word, option_word);
                rated_options.emplace_back(&option_word, ratio, &option);
            }
        }

        return rated_options;
    }
} // namespace simox::alg::detail

namespace simox::alg
{

    /**
     * @brief Fuzzy match `word` to words representing an option.
     *
     * `option_to_words` is a map such as:
     * ```
     * {
     *     "blue": ["blue", "cyan", "azure"],
     *     "violet": ["violet", "purple"],
     * }
     * ```
     * - The keys of `option_to_words` are the options, i.e. the potential results of
     *   matching. One of these keys will be returned.
     * - The values of `option_to_words` are lists of words representing this option.
     *   The `word` is matched against each of these words.
     *
     * `word` is matched against all words of each option,
     * and the option of corresponding to the word with the highest matching ratio
     * wins and is returned.
     *
     * @param word Word to match.
     * @param option_to_words_dict Dictionary mapping options to candidate words.
     * @param match_ratio_fn Function returning a matching ratio between two words.
     * @param log Stream to log a summary to.
     * @return Option whose words have the highest matching ratio against `word`.
     */
    template <class OptionT, class StringT = std::string, class RatioT = double>
    OptionT const*
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RatioT(const StringT& ls, const StringT& rhs)>& match_ratio_fn,
        std::ostream& log)
    {
        using RatedOption = detail::RatedOption<OptionT, StringT, RatioT>;

        std::vector<RatedOption> rated_options =
            detail::fuzzy_multi_match(word, option_to_words, match_ratio_fn);

        if (rated_options.empty())
        {
            return nullptr;
        }

        std::sort(rated_options.begin(), rated_options.end(), RatedOption::compare_reverse);

        const RatedOption& max_element = rated_options.front();

        const StringT& option_word = *max_element.word;
        const RatioT& ratio = max_element.ratio;
        const OptionT& option = *max_element.option;

        log << "Selection option '" << option << "' for word '" << word
            << "' because its candidate word '" << option_word << "' matches '" << word << "' by "
            << std::fixed << std::setprecision(1) << (ratio * 100) << "%"
            << "\nAll option words were:";
        for (const RatedOption& rated_option : rated_options)
        {
            log << "\n- " << std::fixed << std::setprecision(1) << (rated_option.ratio * 100)
                << "%: \t'" << *rated_option.word << "' \t(option '" << *rated_option.option
                << "') ";
        }

        return &option;
    }

    template <class OptionT, class StringT = std::string, class RankT = double>
    OptionT const*
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RankT(const StringT& ls, const StringT& rhs)>& match_ratio_fn)
    {
        using RatedOption = detail::RatedOption<OptionT, StringT, RankT>;

        std::vector<RatedOption> rated_options =
            detail::fuzzy_multi_match(word, option_to_words, match_ratio_fn);

        if (rated_options.empty())
        {
            return nullptr;
        }

        auto max_element =
            std::max_element(rated_options.begin(), rated_options.end(), RatedOption::compare);
        if (max_element == rated_options.end())
        {
            return nullptr;
        }

        const OptionT* option = max_element->option;
        return option;
    }

    template <class OptionT, class StringT = std::string>
    OptionT const*
    fuzzy_multi_match(const StringT& word,
                      const std::map<OptionT, std::vector<StringT>>& option_to_words)
    {
        return fuzzy_multi_match<OptionT, StringT, double>(
            word, option_to_words, fuzzy_match<StringT>);
    }

    template <class OptionT, class StringT = std::string>
    OptionT const*
    fuzzy_multi_match(const StringT& word,
                      const std::map<OptionT, std::vector<StringT>>& option_to_words,
                      std::ostream& log)
    {
        return fuzzy_multi_match<OptionT, StringT, double>(
            word, option_to_words, fuzzy_match<StringT>, log);
    }

} // namespace simox::alg

#endif
