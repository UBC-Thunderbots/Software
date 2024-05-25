#pragma once

#include "software/util/make_enum/make_enum.h"

/**
 * Enums created using the MAKE_REFLECTIVE_ENUM macro will inherit
 * ReflectiveEnum. Having all reflective enums inherit this class
 * enables use of std::is_base_of to assert whether a type is a ReflectiveEnum.
 * 
 * Static virtual methods do not exist in C++ so this class definition is empty; 
 * see the MAKE_REFLECTIVE_ENUM definition for methods supported by ReflectiveEnum.   
 */
class ReflectiveEnum
{
};

/**
 * Creates an "smart" enum with compile-time, template-friendly reflection.
 * 
 * This enum supports
 * - getting the number of enum values
 * - getting a list of all enum values
 *
 * IMPORTANT: This macro does NOT support enums that have manually specified values
 * e.g. MAKE_ENUM(MyEnum, foo=1, bar) is not valid 
 *
 * @param name The name of the enum
 * @param ... The values of the enum
 */
#define MAKE_REFLECTIVE_ENUM(name, ...)                                                         \
    constexpr std::string_view enum_string_args_##name = #__VA_ARGS__;                          \
    static_assert(                                                                              \
        isEnumArgsValid(enum_string_args_##name),                                               \
        "Error: Enums created with the MAKE_REFLECTIVE_ENUM macro may not manually "            \
        "specify enum values. Please remove any enum assignments with the '=' operator");       \
    class name : public ReflectiveEnum                                                          \
    {                                                                                           \
       public:                                                                                  \
        enum Enum                                                                             \
        {                                                                                     \
            __VA_ARGS__                                                                       \
            /* Although common convention, we don't use a COUNT value as the last item in */  \
            /* our enum in order to figure out how many values the enum contains. The */      \
            /* reason for this is that the compiler includes helpful warnings like */         \
            /* "enum value not included in switch" to help avoid missing a case in a */       \
            /* switch statement. Adding an extra COUNT value that will not be handled in */   \
            /* these cases means we will get warnings (or errors). See this stackoverflow */  \
            /* answer for a similar explanation: https://stackoverflow.com/a/2102673 */       \
        }; \
        static size_t numValues()                                                               \
        {                                                                                       \
            return separateEnumStrings(#__VA_ARGS__).size();                                    \
        }                                                                                       \
        static std::vector<Enum> allValues()                                                    \
        {                                                                                       \
            std::vector<Enum> values;                                                           \
            for (unsigned int i = 0; i < name::numValues(); i++)                                \
            {                                                                                   \
                /* This casting relies on the assumption that the enum does not */              \
                /* manually specify any values, because if it did we may cast */                \
                /* to different values or miss some values entirely when */                     \
                /* constructing the vector */                                                   \
                values.emplace_back(static_cast<Enum>(i));                                      \
            }                                                                                   \
            return values;                                                                      \
        }                                                                                       \
    };
