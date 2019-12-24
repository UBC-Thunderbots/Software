#pragma once
#include <magic_enum/magic_enum.hpp>

#include <algorithm>
#include <string>
#include <set>

namespace RobotCapabilities
{
    enum class Capability
    {
        Dribble,
        Kick,
        Chip,
        Move
    };

    /**
     * Returns a set of all capabilities.
     * @return a set of all capabilities
     */
    inline std::set<Capability> allCapabilities()
    {
        return {Capability::Dribble, Capability::Kick, Capability::Chip,
                Capability::Move};
    }
};  // namespace RobotCapabilities

// utility operators below for comparing capabilities

/**
 * Returns true if lhs is a subset of rhs, otherwise false
 * @tparam T the parameter type of lhs and rhs
 * @param lhs an unordered_set
 * @param rhs another unordered_set
 * @return true if lhs is a subset of rhs
 */
template <typename T>
inline bool operator<=(const std::set<T>& lhs, const std::set<T>& rhs)
{
    return std::includes(rhs.begin(), rhs.end(), lhs.begin(), lhs.end());
}

/**
 * Returns true if rhs is a subset of lhs, otherwise false
 * @tparam T the parameter type of lhs and rhs
 * @param lhs an unordered_set
 * @param rhs another unordered_set
 * @return true if rhs is a subset of lhs
 */
template <typename T>
inline bool operator>=(const std::set<T>& lhs, const std::set<T>& rhs)
{
    return std::includes(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

/**
 * Print/stream operator for a robot capability
 * @param os The output stream to print to
 * @param capability The robot capability to print
 * @return The given output stream with the capability printed to it
 */
std::ostream& operator<<(std::ostream& os, const RobotCapabilities::Capability capability){

    std::string capability_name =
        static_cast<std::string>(magic_enum::enum_name(capability));
    os << capability_name;

    return os;
}
