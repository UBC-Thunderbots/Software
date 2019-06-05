#pragma once

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
