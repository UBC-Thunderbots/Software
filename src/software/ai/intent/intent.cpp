#include "software/ai/intent/intent.h"

#include <algorithm>

#include "software/logger/logger.h"

Intent::Intent(unsigned int robot_id, unsigned int priority)
    : robot_id(robot_id), motion_constraints()
{
    setPriority(priority);
}

unsigned int Intent::getPriority(void) const
{
    return priority;
}

unsigned int Intent::getRobotId() const
{
    return robot_id;
}

void Intent::setPriority(unsigned int new_priority)
{
    if (new_priority > 100)
    {
        LOG(WARNING) << "Intent set with out of range priority value: " << new_priority
                     << ". Clamping to range [0, 100]" << std::endl;
        new_priority = std::clamp<unsigned int>(new_priority, 0, 100);
    }
    priority = new_priority;
}

bool Intent::operator==(const Intent &other) const
{
    return this->priority == other.priority && this->robot_id == other.robot_id &&
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
