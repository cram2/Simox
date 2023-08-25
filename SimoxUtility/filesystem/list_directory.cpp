#include "list_directory.h"

#include <algorithm>

namespace simox
{

    namespace impl
    {


        template <class DirectoryIteratorT>
        std::vector<fs::path>
        list_directory(const fs::path& directory, bool local, bool sort)
        {
            std::vector<fs::path> entries;
            for (const auto& entry : DirectoryIteratorT(directory))
            {
                entries.push_back(entry.path());
            }

            if (sort)
            {
                std::sort(entries.begin(), entries.end());
            }

            if (local)
            {
                for (auto& entry : entries)
                {
                    entry = entry.filename();
                }
            }
            return entries;
        }

    } // namespace impl

    std::vector<fs::path>
    fs::list_directory(const path& directory, bool local, bool sort)
    {
        return impl::list_directory<std::filesystem::directory_iterator>(directory, local, sort);
    }

    std::vector<std::filesystem::__cxx11::path>
    fs::list_directory_recursive(const path& directory, bool local, bool sort)
    {
        return impl::list_directory<std::filesystem::recursive_directory_iterator>(
            directory, local, sort);
    }

} // namespace simox
