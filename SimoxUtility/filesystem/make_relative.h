#pragma once

#include <filesystem>

namespace simox::fs
{
    //! Return path when appended to a_From will resolve to same as a_To
    inline std::filesystem::path
    make_relative(std::filesystem::path a_From, std::filesystem::path a_To)
    {
        a_From = std::filesystem::canonical(a_From);
        a_To = std::filesystem::canonical(a_To);
        std::filesystem::path ret;
        std::filesystem::path::const_iterator itrFrom(a_From.begin()), itrTo(a_To.begin());
        // Find common base
        for (std::filesystem::path::const_iterator toEnd(a_To.end()), fromEnd(a_From.end());
             itrFrom != fromEnd && itrTo != toEnd && *itrFrom == *itrTo;
             ++itrFrom, ++itrTo)
            ;
        // Navigate backwards in directory to reach previously found base
        for (std::filesystem::path::const_iterator fromEnd(a_From.end()); itrFrom != fromEnd;
             ++itrFrom)
        {
            if ((*itrFrom) != ".")
                ret /= "..";
        }
        // Now navigate down the directory branch
        //ret.append(itrTo, a_To.end());
        for (; itrTo != a_To.end(); ++itrTo)
            ret /= *itrTo;
        return ret;
    }
} // namespace simox::fs
