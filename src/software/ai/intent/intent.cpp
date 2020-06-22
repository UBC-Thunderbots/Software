#include "software/ai/intent/intent.h"

#include <algorithm>

#include "software/logger/logger.h"

// Implement concrete functions shared by all intents

Intent::Intent(unsigned int priority)
{
    setPriority(priority);
}

unsigned int Intent::getPriority(void) const
{
    return priority;
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
    return this->priority == other.priority;
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
