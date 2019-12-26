#pragma once
#include <algorithm>
#include <set>
#include <string>

namespace RobotCapabilities
{
    enum class Capability
    {
        Dribble,
        Kick,
        Chip,
        Move,
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
 * @param lhs a set of capabilities
 * @param rhs another set of capabilities
 * @return true if lhs is a subset of rhs
 */
bool operator<=(const std::set<RobotCapabilities::Capability>& lhs,
                       const std::set<RobotCapabilities::Capability>& rhs);

/**
 * Returns true if rhs is a subset of lhs, otherwise false
 * @param lhs a set of capabilities
 * @param rhs another set of capabilities
 * @return true if lhs is a subset of rhs
 */
bool operator>=(const std::set<RobotCapabilities::Capability>& lhs,
                       const std::set<RobotCapabilities::Capability>& rhs);

/**
 * Print/stream operator for a robot capability
 * @param os The output stream to print to
 * @param capability The robot capability to print
 * @return The given output stream with the capability printed to it
 */
std::ostream& operator<<(std::ostream& os, const RobotCapabilities::Capability capability);
