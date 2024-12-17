#include "Named.h"

#include <SimoxUtility/error/SimoxError.h>

#include "colormaps.h"

namespace simox::color::cmaps
{

    std::map<std::string, ColorMap> Named::_named;
    Named Named::_instance;

    ColorMap
    Named::get(const std::string& name)
    {
        if (auto it = _named.find(name); it != _named.end())
        {
            return it->second;
        }
        else
        {
            throw simox::error::SimoxError("No ColorMap with name: '" + name + "'.");
        }
    }

    void
    Named::_register(const ColorMap& cmap)
    {
        if (cmap.name().empty())
        {
            throw simox::error::SimoxError("Registering ColorMap without a name.");
        }
        _named[cmap.name()] = cmap;
    }

    void
    Named::_register(const std::string& name, const ColorMap& cmap)
    {
        _named[name] = cmap;
        _named[name].setName(name);
    }

    Named::Named()
    {
        _register_builtin();
    }

    void
    Named::_register_builtin()
    {
        // Perceptually Uniform Sequential
        _register(viridis());
        _register(plasma());
        _register(inferno());
        _register(magma());
        _register(cividis());

        // Sequential
        _register(Greys());
        _register(Purples());
        _register(Blues());
        _register(Greens());
        _register(Oranges());
        _register(Reds());
        _register(YlOrBr());
        _register(YlOrRd());
        _register(OrRd());
        _register(PuRd());
        _register(RdPu());
        _register(BuPu());
        _register(GnBu());
        _register(PuBu());
        _register(YlGnBu());
        _register(PuBuGn());
        _register(BuGn());
        _register(YlGn());

        // Sequential (2)
        _register(binary());
        _register(gray());
        _register(bone());
        _register(pink());
        _register(spring());
        _register(summer());
        _register(autumn());
        _register(winter());
        _register(cool());
        _register(Wistia());
        _register(hot());
        _register(copper());

        // Diverging
        _register(PiYG());
        _register(PRGn());
        _register(BrBG());
        _register(PuOr());
        _register(RdGy());
        _register(RdBu());
        _register(RdYlBu());
        _register(RdYlGn());
        _register(Spectral());
        _register(coolwarm());
        _register(bwr());
        _register(seismic());

        // Cyclic
        _register(twilight());
        _register(twilight_shifted());
        _register(hsv());

        // Qualitative
        _register(Pastel1());
        _register(Pastel2());
        _register(Paired());
        _register(Accent());
        _register(Dark2());
        _register(Set1());
        _register(Set2());
        _register(Set3());
        _register(tab10());
        _register(tab20());
        _register(tab20b());
        _register(tab20c());
    }
} // namespace simox::color::cmaps
