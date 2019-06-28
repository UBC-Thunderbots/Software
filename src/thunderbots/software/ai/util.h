#pragma once
#include <sstream>
#include <unordered_set>

#include "util/logger/init.h"

/**
 * Macro used to define bitwise operations for a class.
 */
#define GEN_BITWISE(CLASS, UNDERLYING)                                                   \
    static constexpr inline CLASS operator|(CLASS lhs, CLASS rhs)                        \
    {                                                                                    \
        return static_cast<CLASS>(static_cast<UNDERLYING>(lhs) |                         \
                                  static_cast<UNDERLYING>(rhs));                         \
    }                                                                                    \
    static inline CLASS& operator|=(CLASS& lhs, CLASS rhs)                               \
    {                                                                                    \
        lhs = static_cast<CLASS>(static_cast<UNDERLYING>(lhs) |                          \
                                 static_cast<UNDERLYING>(rhs));                          \
        return lhs;                                                                      \
    }                                                                                    \
    static constexpr inline CLASS operator&(CLASS lhs, CLASS rhs)                        \
    {                                                                                    \
        return static_cast<CLASS>(static_cast<UNDERLYING>(lhs) &                         \
                                  static_cast<UNDERLYING>(rhs));                         \
    }                                                                                    \
    static inline CLASS& operator&=(CLASS& lhs, CLASS rhs)                               \
    {                                                                                    \
        lhs = static_cast<CLASS>(static_cast<UNDERLYING>(lhs) &                          \
                                 static_cast<UNDERLYING>(rhs));                          \
        return lhs;                                                                      \
    }                                                                                    \
    static constexpr inline CLASS operator^(CLASS lhs, CLASS rhs)                        \
    {                                                                                    \
        return static_cast<CLASS>(static_cast<UNDERLYING>(lhs) ^                         \
                                  static_cast<UNDERLYING>(rhs));                         \
    }                                                                                    \
    static inline CLASS& operator^=(CLASS& lhs, CLASS rhs)                               \
    {                                                                                    \
        lhs = static_cast<CLASS>(static_cast<UNDERLYING>(lhs) ^                          \
                                 static_cast<UNDERLYING>(rhs));                          \
        return lhs;                                                                      \
    }                                                                                    \
    static constexpr inline CLASS operator~(CLASS lhs)                                   \
    {                                                                                    \
        return static_cast<CLASS>(~static_cast<UNDERLYING>(lhs));                        \
    }

inline std::unordered_set<unsigned int> commaSeparatedListToSet(
    const std::string& list_str)
{
    std::stringstream ss(list_str);
    std::unordered_set<unsigned int> output;
    std::string list_item_str;
    while (std::getline(ss, list_item_str, ','))
    {
        try
        {
            output.emplace(std::stoi(list_item_str));
        }
        catch (const std::invalid_argument& e)
        {
            LOG(WARNING)
                << "There are symbols other than numbers, spaces, and commas in the comma-separated list \""
                << list_str << "\"";
        }
    }
    return output;
}