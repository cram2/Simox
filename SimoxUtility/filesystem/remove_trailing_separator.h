#pragma once

#include <filesystem>

namespace simox::fs
{
    using namespace std::filesystem;

    inline fs::path
    remove_trailing_separator(fs::path p)
    {
        p /= "dummy";
        return p.parent_path();
    }
} // namespace simox::fs
