#include "json.h"

namespace simox
{

    void
    color::to_json(nlohmann::json& j, const Color& color)
    {
        j["r"] = color.r;
        j["g"] = color.g;
        j["b"] = color.b;
        j["a"] = color.a;
    }

    void
    color::from_json(const nlohmann::json& j, Color& color)
    {
        j.at("r").get_to(color.r);
        j.at("g").get_to(color.g);
        j.at("b").get_to(color.b);
        j.at("a").get_to(color.a);
    }

} // namespace simox
