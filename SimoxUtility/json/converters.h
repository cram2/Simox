#pragma once

#include <map>
#include <string>

#include <eigen3/Eigen/Core>

#include "json.h"

namespace simox::json
{
    Eigen::Matrix4f posquat2eigen4f(const std::string& str);
    Eigen::Matrix4f posquat2eigen4f(const char* str);
    Eigen::Matrix4f posquat2eigen4f(const simox::json::json& j);

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const std::string& str);
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const char* str);
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const simox::json::json& j);

    std::string eigen4f2posquatJson(const Eigen::Matrix4f& str);
    std::string eigen4fVector2posquatArrayJson(const std::vector<Eigen::Matrix4f>& str);

    std::map<std::string, float> json2NameValueMap(const std::string& str);
    std::map<std::string, float> json2NameValueMap(const char* str);
    std::map<std::string, float> json2NameValueMap(const simox::json::json& j);
} // namespace simox::json
