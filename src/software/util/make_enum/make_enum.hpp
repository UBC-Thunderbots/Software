#pragma once

#include "software/util/make_enum/make_enum_helpers.hpp"

/**
 * Creates a "smart" enum with compile-time, template-friendly reflection.
 *
 * This enum will have support for:
 * 1. Getting the number of enum values (reflective_enum::size)
 * 2. Getting the string representation of each enum value (reflective_enum::valueNames)
 * 3. Getting a list of all enum values (reflective_enum::values)
 * 4. Getting an enum value from its string representation (reflective_enum::fromName)
 *
 * IMPORTANT: This macro does NOT support enums that have manually specified values.
 * e.g. MAKE_ENUM(MyEnum, foo=1, bar) is not valid
 *
 * @param name The name of the enum
 * @param ... The values of the enum
 */
#define MAKE_ENUM(name, ...)                                                               \
    constexpr std::string_view _enum_string_args_##name = #__VA_ARGS__;                    \
    static_assert(                                                                         \
        isEnumArgsValid(_enum_string_args_##name),                                         \
        "Error: Enums created with the MAKE_ENUM macro may not manually specify enum "     \
        "values. Please remove any enum assignments with the '=' operator");               \
    enum class name                                                                      \
    {                                                                                    \
        __VA_ARGS__                                                                      \
        /* Although common convention, we don't use a COUNT value as the last item in */ \
        /* our enum in order to figure out how many values the enum contains. The */     \
        /* reason for this is that the compiler includes helpful warnings like */        \
        /* "enum value not included in switch" to help avoid missing a case in a */      \
        /* switch statement. Adding an extra COUNT value that will not be handled in */  \
        /* these cases means we will get warnings (or errors). See this stackoverflow */ \
        /* answer for a similar explanation: https://stackoverflow.com/a/2102673 */      \
    }; \
    constexpr size_t _enum_size_##name = countSubstrings(_enum_string_args_##name, ',');   \
    constexpr auto _enum_values_##name = getEnumValues<name, _enum_size_##name>();         \
    constexpr auto _enum_value_names_##name = splitString(                                 \
        _enum_string_args_##name, ',', std::make_index_sequence<_enum_size_##name>());     \
    template <>                                                                            \
    struct reflective_enum::is_reflective_enum<name> : std::true_type                      \
    {                                                                                      \
    };                                                                                     \
    template <>                                                                            \
    constexpr size_t reflective_enum::size<name>()                                         \
    {                                                                                      \
        return _enum_size_##name;                                                          \
    }                                                                                      \
    template <>                                                                            \
    constexpr auto reflective_enum::values<name>()                                         \
    {                                                                                      \
        return _enum_values_##name;                                                        \
    }                                                                                      \
    template <>                                                                            \
    constexpr auto reflective_enum::valueNames<name>()                                     \
    {                                                                                      \
        return _enum_value_names_##name;                                                   \
    }                                                                                      \
    inline std::ostream& operator<<(std::ostream& os, name value)                          \
    {                                                                                      \
        /* This index lookup relies on the assumption that the enum does not manually */   \
        /* specify any values. If it did, the underlying integer of the given value */     \
        /* may be out of range of the vector of strings */                                 \
        os << reflective_enum::valueNames<name>().at(static_cast<int>(value));             \
        return os;                                                                         \
    }

namespace reflective_enum
{
    /**
     * Type trait that checks whether T is a reflective enum, i.e. whether it
     * was created using the MAKE_ENUM macro.
     */
    template <typename T>
    struct is_reflective_enum : std::false_type
    {
    };

    /**
     * Concept that is satisfied by any type E that is a reflective enum.
     */
    template <typename E>
    concept ReflectiveEnum = is_reflective_enum<E>::value;

    /**
     * Returns the number of values in the reflective enum E.
     *
     * @return the number of values in the enum
     */
    template <ReflectiveEnum E>
    constexpr size_t size();

    /**
     * Returns an array containing the values of the reflective enum E.
     *
     * @return an array with the values of the enum
     */
    template <ReflectiveEnum E>
    constexpr auto values();

    /**
     * Returns an array contain the string representations of each value
     * in the reflective enum E.
     *
     * @return an array with the names of the values in the enum
     */
    template <ReflectiveEnum E>
    constexpr auto valueNames();

    /**
     * Returns the enum value with the given string representation.
     *
     * @param value_name the string representation of the enum value
     *
     * @return the enum value
     */
    template <ReflectiveEnum E>
    constexpr E fromName(const std::string value_name)
    {
        constexpr size_t enum_size      = size<E>();
        constexpr auto enum_value_names = valueNames<E>();
        for (size_t i = 0; i < enum_size; ++i)
        {
            if (enum_value_names.at(i) == value_name)
            {
                return static_cast<E>(i);
            }
        }
        throw std::invalid_argument(value_name + " cannot be converted to enum value");
    }
}  // namespace reflective_enum
