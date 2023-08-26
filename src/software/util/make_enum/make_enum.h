#pragma once

#include <algorithm>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

/**
 * Checks if a string representing enum values is valid.
 *
 * An enum string is valid if it does not contain any '=', meaning
 * the enum is not being assigned any values manually
 *
 * @param str A string containing comma-separated enum value names
 *
 * @return True if the string is valid, and false otherwise
 */
constexpr bool isEnumArgsValid(const std::string_view& str)
{
    size_t index = str.find('=');
    return index == std::string_view::npos;
}

/**
 * Separates a string of comma-separated values into a vector
 *
 * @param enum_string A string of comma-separated enum values
 *
 * @return A vector of strings, where each string is a value from the original string
 */
std::vector<std::string> separateEnumStrings(std::string enum_string);

/**
 * Creates an enum.
 *
 * This enum will have support for
 * 1. Getting the number of enum values
 * 2. Getting the string representation of each enum value
 * 3. Getting a list of all enum values
 *
 * IMPORTANT: This macro does NOT support enums that have manually specified values
 * Eg.
 * MAKE_ENUM(MyEnum, getTrajPartAndDeltaTime=1, bar)
 * is not valid
 *
 * @param name The name of the enum
 * @param ... The values of the enum
 */
#define MAKE_ENUM(name, ...)                                                               \
    constexpr std::string_view enum_string_args_##name = #__VA_ARGS__;                     \
    static_assert(                                                                         \
        isEnumArgsValid(enum_string_args_##name),                                          \
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
    static inline std::vector<std::string> allStringValues##name()                         \
    {                                                                                      \
        return separateEnumStrings(#__VA_ARGS__);                                          \
    }                                                                                      \
    inline std::ostream& operator<<(std::ostream& os, name value)                          \
    {                                                                                      \
        std::vector<std::string> enum_strings = allStringValues##name();                   \
        /* This index lookup relies on the assumption that the enum does not manually */   \
        /* specify any values. If it did, the underlying integer of the given value */     \
        /* may be out of range of the vector of strings */                                 \
        os << enum_strings.at(static_cast<int>(value));                                    \
        return os;                                                                         \
    }                                                                                      \
    static inline size_t size##name()                                                      \
    {                                                                                      \
        std::vector<std::string> enum_strings = allStringValues##name();                   \
        return enum_strings.size();                                                        \
    }                                                                                      \
    static inline name fromStringTo##name(std::string str_in)                              \
    {                                                                                      \
        std::vector<std::string> enum_strings = allStringValues##name();                   \
        for (size_t i = 0; i < enum_strings.size(); i++)                                   \
        {                                                                                  \
            if (str_in == enum_strings[i])                                                 \
            {                                                                              \
                return static_cast<name>(i);                                               \
            }                                                                              \
        }                                                                                  \
        throw std::invalid_argument(str_in + " cannot be converted to " + #name);          \
    }                                                                                      \
    static inline std::vector<name> allValues##name()                                      \
    {                                                                                      \
        std::vector<name> values;                                                          \
        for (unsigned int i = 0; i < size##name(); i++)                                    \
        {                                                                                  \
            /* This casting relies on the assumption that the enum does not manually */    \
            /* specify any values, because if it did we may cast to different values */    \
            /* or miss some values entirely when constructing the vector */                \
            values.emplace_back(static_cast<name>(i));                                     \
        }                                                                                  \
        return values;                                                                     \
    }

template <typename T>
std::string toString(const T& value)
{
    std::ostringstream ss;
    ss << value;
    return ss.str();
}
