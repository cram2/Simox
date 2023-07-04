#pragma once

#include <filesystem>
#include <vector>

namespace simox::fs
{
    using namespace std::filesystem;

    /**
     * @brief List the entries in directory `directory`.
     * @param directory Path to the directory.
     * @param local
     *      If true, returned paths are relative to `directory`.
     *      If false, the entries are prepended by `directory`.
     * @param sort If true, entries are sorted alphatically.
     * @return The entries in `directory`.
     */
    std::vector<path> list_directory(const path& directory, bool local = false, bool sort = true);

    /**
     * @brief Same as list_directory(), but recurses into subdirectories.
     */
    std::vector<path>
    list_directory_recursive(const path& directory, bool local = false, bool sort = true);

} // namespace simox::fs
