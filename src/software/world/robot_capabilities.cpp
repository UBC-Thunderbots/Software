#include "software/world/robot_capabilities.h"

#include <magic_enum/magic_enum.hpp>

inline bool operator<=(const std::set<RobotCapabilities::Capability>& lhs,
                       const std::set<RobotCapabilities::Capability>& rhs)
{
    return std::includes(rhs.begin(), rhs.end(), lhs.begin(), lhs.end());
}

/**
 * Returns true if rhs is a subset of lhs, otherwise false
 * @param lhs a set of capabilities
 * @param rhs another set of capabilities
 * @return true if lhs is a subset of rhs
 */
inline bool operator>=(const std::set<RobotCapabilities::Capability>& lhs,
                       const std::set<RobotCapabilities::Capability>& rhs)
{
    return std::includes(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

/**
 * Print/stream operator for a robot capability
 * @param os The output stream to print to
 * @param capability The robot capability to print
 * @return The given output stream with the capability printed to it
 */
std::ostream& operator<<(std::ostream& os, const RobotCapabilities::Capability capability)
{
    std::string capability_name =
        static_cast<std::string>(magic_enum::enum_name(capability));
    os << capability_name;

    return os;
}
