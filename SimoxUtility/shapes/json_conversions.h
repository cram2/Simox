#pragma once

#include <SimoxUtility/json/json.hpp>

namespace simox
{
    class AxisAlignedBoundingBox;
    template <class FloatT>
    class OrientedBox;


    void to_json(nlohmann::json& j, const AxisAlignedBoundingBox& aabb);
    void from_json(const nlohmann::json& j, AxisAlignedBoundingBox& aabb);

    void to_json(nlohmann::json& j, const OrientedBox<float>& ob);
    void from_json(const nlohmann::json& j, OrientedBox<float>& ob);

    void to_json(nlohmann::json& j, const OrientedBox<double>& ob);
    void from_json(const nlohmann::json& j, OrientedBox<double>& ob);

} // namespace simox
