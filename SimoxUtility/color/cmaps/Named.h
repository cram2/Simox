#pragma once

#include <map>
#include <string>

#include <SimoxUtility/color/ColorMap.h>

namespace simox::color::cmaps
{

    /**
     * @brief Registry for named color maps.
     *
     * Source: https://matplotlib.org/3.1.0/tutorials/colors/colormaps.html
     */
    class Named
    {
    public:
        /// Get a color map by name.
        /// @throws `simox::error::SimoxError` If there is no color map with that name.
        static ColorMap get(const std::string& name);

        /// Indicate whether there is a named color map with the given name.
        static bool
        has(const std::string& name)
        {
            return _named.count(name) > 0;
        }

        /// Get all  registered color maps.
        inline static const std::map<std::string, ColorMap>&
        all()
        {
            return _named;
        }

        /// Register a colormap.
        /// @throws `simox::error::SimoxError` If `cmap`'s name is empty.
        static void _register(const ColorMap& cmap);
        /// Register a colormap under given name.
        static void _register(const std::string& name, const ColorMap& cmap);

    private:
        /// The registered color maps.
        static std::map<std::string, ColorMap> _named;


    private:
        /// Private constructor. Calls `_register_builtin().
        Named();
        /// The private instance.
        static Named _instance;
        /// Register builtin color maps.
        static void _register_builtin();
    };

    /// @brief Get a color map by name.
    /// @throws `simox::error::SimoxError` If there is no color map with that name.
    inline ColorMap
    get_named(const std::string& name)
    {
        return Named::get(name);
    }

    /// Indicate whether there is a named color map with the given name.
    inline bool
    has_named(const std::string& name)
    {
        return Named::has(name);
    }

} // namespace simox::color::cmaps
