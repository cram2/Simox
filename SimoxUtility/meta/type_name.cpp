#include "type_name.h"

#include <boost/core/demangle.hpp>

std::string
simox::meta::get_type_name(const std::type_info& typeInfo)
{
    return boost::core::demangle(typeInfo.name());
}
