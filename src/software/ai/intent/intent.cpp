#include "software/ai/intent/intent.h"

#include <algorithm>

#include "software/logger/logger.h"

Intent::Intent(unsigned int robot_id) : robot_id(robot_id), motion_constraints() {}

unsigned int Intent::getRobotId() const
{
    return robot_id;
}

bool Intent::operator==(const Intent &other) const
{
    return this->robot_id == other.robot_id &&
           motion_constraints == other.motion_constraints;
}

bool Intent::operator!=(const Intent &other) const
{
    return !((*this) == other);
}

void Intent::setMotionConstraints(const std::set<MotionConstraint> &motion_constraints)
{
    this->motion_constraints = motion_constraints;
}

std::set<MotionConstraint> Intent::getMotionConstraints(void) const
{
    return motion_constraints;
}
