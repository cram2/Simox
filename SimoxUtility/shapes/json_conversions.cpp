#include "json_conversions.h"

#include <SimoxUtility/json.h>
#include <SimoxUtility/json/util.h>

#include "AxisAlignedBoundingBox.h"
#include "OrientedBox.h"

void
simox::to_json(nlohmann::json& j, const simox::AxisAlignedBoundingBox& aabb)
{
    j["center"] = aabb.center();
    j["extents"] = aabb.extents();
    j["min"] = aabb.min();
    j["max"] = aabb.max();
}

void
simox::from_json(const nlohmann::json& j, simox::AxisAlignedBoundingBox& aabb)
{
    if (j.count("min") && j.count("max"))
    {
        auto min = j.at("min").get<Eigen::Vector3f>();
        auto max = j.at("max").get<Eigen::Vector3f>();

        aabb.set_min(min);
        aabb.set_max(max);
    }
    else
    {
        Eigen::Vector3f center =
            simox::json::get_at_any_key<Eigen::Vector3f>(j, {"center", "pos", "position"});
        Eigen::Vector3f extents =
            simox::json::get_at_any_key<Eigen::Vector3f>(j, {"extents", "dimensions"});

        aabb.set_center(center);
        aabb.set_extents(extents);
    }
}

namespace simox::json
{
    template <typename Float>
    void
    to_json(nlohmann::json& j, const OrientedBox<Float>& ob)
    {
        j["position"] = ob.center();
        j["orientation"] = Eigen::Quaternion<Float>(ob.rotation());
        j["extents"] = ob.dimensions();
    }

    template <typename Float>
    void
    from_json(const nlohmann::json& j, OrientedBox<Float>& ob)
    {
        Eigen::Vector3d pos = simox::json::get_at_any_key<Eigen::Vector3d>(j, {"pos", "position"});
        Eigen::Quaterniond quat =
            simox::json::get_at_any_key<Eigen::Quaterniond>(j, {"ori", "orientation"});
        Eigen::Matrix3d ori = quat.toRotationMatrix();
        Eigen::Vector3d extents =
            simox::json::get_at_any_key<Eigen::Vector3d>(j, {"extents", "dimensions"});

        Eigen::Vector3d corner = pos - ori * extents / 2;

        ob = OrientedBox<double>(
                 corner, ori.col(0) * extents(0), ori.col(1) * extents(1), ori.col(2) * extents(2))
                 .cast<Float>();
    }
} // namespace simox::json

void
simox::to_json(nlohmann::json& j, const OrientedBox<float>& ob)
{
    simox::json::to_json<float>(j, ob);
}

void
simox::from_json(const nlohmann::json& j, OrientedBox<float>& ob)
{
    simox::json::from_json<float>(j, ob);
}

void
simox::to_json(nlohmann::json& j, const OrientedBox<double>& ob)
{
    simox::json::to_json<double>(j, ob);
}

void
simox::from_json(const nlohmann::json& j, OrientedBox<double>& ob)
{
    simox::json::from_json<double>(j, ob);
}
