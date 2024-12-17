#pragma once

namespace simox
{
    template <class InputIt, class DecisionFunc, class IfTrueFunc, class IfFalseFunc>
    std::size_t
    for_each_if(InputIt first, InputIt last, DecisionFunc d, IfTrueFunc t, IfFalseFunc f)
    {
        std::size_t numTrue = 0;
        for (; first != last; ++first)
        {
            if (d(*first))
            {
                ++numTrue;
                t(*first);
            }
            else
            {
                f(*first);
            }
        }
        return numTrue;
    }

    template <class Container, class DecisionFunc, class IfTrueFunc, class IfFalseFunc>
    std::size_t
    for_each_if(const Container& cont, DecisionFunc d, IfTrueFunc t, IfFalseFunc f)
    {
        return for_each_if(cont.begin(), cont.end(), std::move(d), std::move(t), std::move(f));
    }
} // namespace simox
