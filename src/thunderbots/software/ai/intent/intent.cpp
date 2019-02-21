#include "intent.h"

// Implement concrete functions shared by all intents

int Intent::getPriority(void) const
{
    return priority;
}

void Intent::setPriority(int new_priority)
{
    priority = new_priority;
}

bool Intent::operator==(const Intent &other) const {
    return this->priority == other.priority;
}

bool Intent::operator!=(const Intent &other) const {
    return !((*this) == other);
}
