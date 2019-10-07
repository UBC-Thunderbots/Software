#include "software/ai/motion_constraint/motion_constraint.h"

#include <map>
#include <string>

std::ostream& operator<<(std::ostream& os, const MotionConstraint& constraint)
{
    static const std::map<MotionConstraint, std::string> motion_constraint_names = {
        {MotionConstraint::ENEMY_ROBOTS_COLLISION, "ENEMY_ROBOTS_COLLISION"},
        {MotionConstraint::FRIENDLY_DEFENSE_AREA, "FRIENDLY_DEFENSE_AREA"},
        {MotionConstraint::ENEMY_DEFENSE_AREA, "ENEMY_DEFENSE_AREA"},
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, "INFLATED_ENEMY_DEFENSE_AREA"},
        {MotionConstraint::CENTER_CIRCLE, "CENTER_CIRCLE"},
        {MotionConstraint::HALF_METER_AROUND_BALL, "HALF_METER_AROUND_BALL"},
        {MotionConstraint::ENEMY_HALF, "ENEMY_HALF"},
        {MotionConstraint::FRIENDLY_HALF, "FRIENDLY_HALF"}};

    auto iter = motion_constraint_names.find(constraint);
    if (iter != motion_constraint_names.end())
    {
        os << iter->second;
    }
    else
    {
        os << "No String Representation For MotionConstraint: " << static_cast<int>(constraint);
    }
    return os;
}
