#pragma once

#include <string>
#include <typeinfo>

namespace simox::meta
{

    /// Return the demangled type name in `typeInfo`.
    std::string get_type_name(const std::type_info& typeInfo);

    /// Return the demangled (static) type name of `t`.
    template <typename T>
    std::string
    get_type_name()
    {
        return get_type_name(typeid(T));
    }

    /// Return the demangled (dynamic) type name of `t`.
    template <typename T>
    std::string
    get_type_name(const T& t)
    {
        return get_type_name(typeid(t));
    }

} // namespace simox::meta
