#pragma once

#ifdef DIFFLIB_FOUND

#include <iomanip>
#include <optional>

#include <SimoxUtility/algorithm/string/fuzzy_match.hpp>

namespace simox::alg
{
    /**
     * @brief A matched option.
     */
    template <class OptionT, class StringT = std::string, class RatioT = double>
    struct MatchedOption
    {
        /**
         * @brief The option.
         */
        const OptionT* option;
        /**
         * @brief The word representing the option.
         */
        const StringT* word;
        /**
         * @brief The match ratio in [0, 1] between the option word and the probed word.
         */
        RatioT ratio;

        /**
         * @brief Comare ratios ascendingly.
         */
        static bool
        compare(const MatchedOption& lhs, const MatchedOption& rhs)
        {
            return lhs.ratio < rhs.ratio;
        }

        /**
         * @brief Comare ratios descendingly.
         */
        static bool
        compare_reverse(const MatchedOption& lhs, const MatchedOption& rhs)
        {
            return lhs.ratio > rhs.ratio;
        }
    };
} // namespace simox::alg

namespace simox::alg::detail
{

    template <class OptionT, class StringT = std::string, class RatioT = double>
    std::vector<MatchedOption<OptionT, StringT, RatioT>>
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RatioT(const StringT& ls, const StringT& rhs)>& matchRatioFn)
    {
        using MatchedOption = MatchedOption<OptionT, StringT, RatioT>;
        std::vector<MatchedOption> matched_options;

        for (const auto& [option, option_words] : option_to_words)
        {
            for (const auto& option_word : option_words)
            {
                RatioT ratio = matchRatioFn(word, option_word);
                matched_options.push_back(MatchedOption{
                    .option = &option,
                    .word = &option_word,
                    .ratio = ratio,
                });
            }
        }

        return matched_options;
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
     * @return
     *   Option whose words have the highest matching ratio against `word`,
     *   or `std::nullopt` if there are no options.
     */
    template <class OptionT, class StringT = std::string, class RatioT = double>
    std::optional<MatchedOption<OptionT, StringT, RatioT>>
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RatioT(const StringT& ls, const StringT& rhs)>& match_ratio_fn,
        std::ostream& log)
    {
        using MatchedOption = MatchedOption<OptionT, StringT, RatioT>;

        std::vector<MatchedOption> matched_options =
            detail::fuzzy_multi_match(word, option_to_words, match_ratio_fn);

        if (matched_options.empty())
        {
            return std::nullopt;
        }

        std::sort(matched_options.begin(), matched_options.end(), MatchedOption::compare_reverse);

        const MatchedOption& max_element = matched_options.front();

        const StringT& option_word = *max_element.word;
        const RatioT& ratio = max_element.ratio;
        const OptionT& option = *max_element.option;

        log << "Selecting option '" << option << "' for word '" << word
            << "' because its candidate word '" << option_word << "' matches '" << word << "' by "
            << std::fixed << std::setprecision(1) << (ratio * 100) << "%"
            << "\nAll option words were:";
        for (const MatchedOption& matched_option : matched_options)
        {
            log << "\n- " << std::fixed << std::setprecision(1) << (matched_option.ratio * 100)
                << "%: \t'" << *matched_option.word << "' \t(option '" << *matched_option.option
                << "') ";
        }

        return max_element;
    }

    template <class OptionT, class StringT = std::string, class RatioT = double>
    std::optional<MatchedOption<OptionT, StringT, RatioT>>
    fuzzy_multi_match(
        const StringT& word,
        const std::map<OptionT, std::vector<StringT>>& option_to_words,
        const std::function<RatioT(const StringT& ls, const StringT& rhs)>& match_ratio_fn)
    {
        using MatchedOption = MatchedOption<OptionT, StringT, RatioT>;

        std::vector<MatchedOption> matched_options =
            detail::fuzzy_multi_match(word, option_to_words, match_ratio_fn);

        if (matched_options.empty())
        {
            return std::nullopt;
        }

        auto max_element = std::max_element(
            matched_options.begin(), matched_options.end(), MatchedOption::compare);
        if (max_element == matched_options.end())
        {
            return std::nullopt;
        }

        return *max_element;
    }

    template <class OptionT, class StringT = std::string>
    std::optional<MatchedOption<OptionT, StringT, double>>
    fuzzy_multi_match(const StringT& word,
                      const std::map<OptionT, std::vector<StringT>>& option_to_words)
    {
        return fuzzy_multi_match<OptionT, StringT, double>(
            word, option_to_words, fuzzy_match<StringT>);
    }

    template <class OptionT, class StringT = std::string>
    std::optional<MatchedOption<OptionT, StringT, double>>
    fuzzy_multi_match(const StringT& word,
                      const std::map<OptionT, std::vector<StringT>>& option_to_words,
                      std::ostream& log)
    {
        return fuzzy_multi_match<OptionT, StringT, double>(
            word, option_to_words, fuzzy_match<StringT>, log);
    }

} // namespace simox::alg

#endif
