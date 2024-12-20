#include "converters.h"

#include <SimoxUtility/math/convert.h>

namespace simox
{
    Eigen::Matrix4f
    json::posquat2eigen4f(const std::string& str)
    {
        return posquat2eigen4f(::simox::json::json::parse(str));
    }

    Eigen::Matrix4f
    json::posquat2eigen4f(const char* str)
    {
        return posquat2eigen4f(::simox::json::json::parse(str));
    }

    Eigen::Matrix4f
    json::posquat2eigen4f(const nlohmann::json& j)
    {
        return simox::math::pos_quat_to_mat4f(j.at("x").get<float>(),
                                              j.at("y").get<float>(),
                                              j.at("z").get<float>(),
                                              Eigen::Quaternionf(j.at("qw").get<float>(),
                                                                 j.at("qx").get<float>(),
                                                                 j.at("qy").get<float>(),
                                                                 j.at("qz").get<float>()));
    }

    std::vector<Eigen::Matrix4f>
    json::posquatArray2eigen4fVector(const std::string& str)
    {
        return posquatArray2eigen4fVector(::simox::json::json::parse(str));
    }

    std::vector<Eigen::Matrix4f>
    json::posquatArray2eigen4fVector(const char* str)
    {
        return posquatArray2eigen4fVector(::simox::json::json::parse(str));
    }

    std::vector<Eigen::Matrix4f>
    json::posquatArray2eigen4fVector(const nlohmann::json& j)
    {
        if (!j.is_array())
        {
            throw std::invalid_argument{"posquatArray2eigen4fVector: json has to be an array"};
        }
        std::vector<Eigen::Matrix4f> result;
        result.reserve(j.size());
        for (const auto& element : j)
        {
            result.emplace_back(posquat2eigen4f(element));
        }
        return result;
    }

    std::string
    json::eigen4f2posquatJson(const Eigen::Matrix4f& mx)
    {
        const Eigen::Quaternionf quat = simox::math::mat4f_to_quat(mx);
        nlohmann::json j = {
            {"qx", quat.x()},
            {"qy", quat.y()},
            {"qz", quat.z()},
            {"qw", quat.w()},

            {"x", mx(0, 3)},
            {"y", mx(1, 3)},
            {"z", mx(2, 3)},
        };
        return j.dump(4);
    }

    std::string
    json::eigen4fVector2posquatArrayJson(const std::vector<Eigen::Matrix4f>& vec)
    {
        nlohmann::json jar = nlohmann::json::array();
        for (const auto& mx : vec)
        {
            const Eigen::Quaternionf quat = simox::math::mat4f_to_quat(mx);
            nlohmann::json j = {
                {"qx", quat.x()},
                {"qy", quat.y()},
                {"qz", quat.z()},
                {"qw", quat.w()},

                {"x", mx(0, 3)},
                {"y", mx(1, 3)},
                {"z", mx(2, 3)},
            };
            jar.push_back(j);
        }
        return jar.dump(4);
    }

    std::map<std::string, float>
    json::json2NameValueMap(const std::string& str)
    {
        return json2NameValueMap(::simox::json::json::parse(str));
    }

    std::map<std::string, float>
    json::json2NameValueMap(const char* str)
    {
        return json2NameValueMap(::simox::json::json::parse(str));
    }

    std::map<std::string, float>
    json::json2NameValueMap(const nlohmann::json& j)
    {
        if (!j.is_object())
        {
            throw std::invalid_argument{"json2NameValueMap: json has to be an object"};
        }
        return j.get<std::map<std::string, float>>();
    }
} // namespace simox
