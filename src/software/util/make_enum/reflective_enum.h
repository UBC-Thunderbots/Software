#include "software/util/make_enum/make_enum.h"

class ReflectiveEnum;

#define MAKE_REFLECTIVE_ENUM(name, ...)                                                         \
    class name : public ReflectiveEnum                                                          \
    {                                                                                           \
        constexpr std::string_view enum_string_args = #__VA_ARGS__;                             \
        static_assert(                                                                          \
            isEnumArgsValid(enum_string_args_##name),                                           \
            "Error: Enums created with the MAKE_REFLECTIVE_ENUM macro may not manually "        \
            "specify enum values. Please remove any enum assignments with the '=' operator");   \
       public:                                                                                  \
        enum Enum                                                                               \
        {                                                                                       \
            __VA_ARGS__                                                                         \
        /* Although common convention, we don't use a COUNT value as the last item in */        \
        /* our enum in order to figure out how many values the enum contains. The */            \
        /* reason for this is that the compiler includes helpful warnings like */               \
        /* "enum value not included in switch" to help avoid missing a case in a */             \
        /* switch statement. Adding an extra COUNT value that will not be handled in */         \
        /* these cases means we will get warnings (or errors). See this stackoverflow */        \
        /* answer for a similar explanation: https://stackoverflow.com/a/2102673 */             \
        }                                                                                       \
        static size_t size()                                                                    \
        {                                                                                       \
            return separateEnumStrings(#__VA_ARGS__).size();                                    \
        }                                                                                       \
        static std::vector<Enum> allValues()                                                    \
        {                                                                                       \
            std::vector<Enum> values;                                                           \
            for (unsigned int i = 0; i < size(); i++)                                           \
            {                                                                                   \
                /* This casting relies on the assumption that the enum does not manually */     \                                                                       \
                /* specify any values, because if it did we may cast to different values */     \                                                                       \
                /* or miss some values entirely when constructing the vector */                 \
                values.emplace_back(static_cast<Enum>(i));                                      \
            }                                                                                   \
            return values;                                                                      \
        }                                                                                       \
    };