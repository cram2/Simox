#pragma once

namespace simox::meta
{
    /**
     * @brief Whether a type T is the instance of a given template Template. (this is the case for false)
     */
    template <template <class...> class Template, class T>
    struct is_instantiation_of : std::false_type
    {
    };

    /**
     * @brief Whether a type T is the instance of a given template Template. (this is the case for true)
     */
    template <template <class...> class Template, class... Params>
    struct is_instantiation_of<Template, Template<Params...>> : std::true_type
    {
    };

    template <template <class...> class Template, class T>
    constexpr bool is_instantiation_of_v = is_instantiation_of<Template, T>::value;

    template <class T, template <class...> class... Templates>
    constexpr bool is_instantiation_of_any_v = (is_instantiation_of_v<Templates, T> || ...);
} // namespace simox::meta
