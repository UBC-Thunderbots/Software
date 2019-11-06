#include "software/ai/motion_constraint/motion_constraint.h"

#include <magic_enum/magic_enum.hpp>
#include <string>

std::ostream& operator<<(std::ostream& os, const MotionConstraint& constraint)
{
    std::string find_motion_constraint_name_result =
        static_cast<std::string>(magic_enum::enum_name(constraint));
    os << find_motion_constraint_name_result;

    return os;
}
