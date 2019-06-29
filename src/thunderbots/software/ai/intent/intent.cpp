#include "intent.h"

#include <algorithm>

#include "util/logger/init.h"

// Implement concrete functions shared by all intents

Intent::Intent(unsigned int priority) : areas_to_avoid({})
{
    setPriority(priority);
}

unsigned int Intent::getPriority(void) const
{
    return priority;
}

void Intent::setPriority(unsigned int new_priority)
{
    if (new_priority < 0 || new_priority > 100)
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

std::vector<AvoidArea> Intent::getAreasToAvoid() const
{
    return areas_to_avoid;
}

void Intent::setAreasToAvoid(const std::vector<AvoidArea> &areas_to_avoid)
{
    this->areas_to_avoid = areas_to_avoid;
}
