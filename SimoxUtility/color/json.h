#pragma once

#include <SimoxUtility/json/json.hpp>

#include "Color.h"

namespace simox::color
{
    void to_json(nlohmann::json& j, const Color& color);
    void from_json(const nlohmann::json& j, Color& color);
} // namespace simox::color
