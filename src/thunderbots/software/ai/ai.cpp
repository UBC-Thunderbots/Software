#include "ai.h"

AI::AI() : rrt_navigator(), stp_high_level()
{
}

std::vector<std::unique_ptr<Primitive>> AI::getPrimitives(
    const AITimestamp &timestamp) const
{
    // TODO: The only thing the AI really needs an updated timestamp for (updated relative
    // to the timestamp when the state/world was last updated) is to update the
    // "zero-time"
    // for any predictors (ie. modules that predict Robot, Ball position etc.). When those
    // are implemented we can update them in some way from here, if necessary.

    // Use generic variables with the types of the Abstract base clases
    // so we can only operate in terms of the public interfaces of these modules
    //
    // These can't be declared in the header because the default copy
    // constructor for the class will be automatically deleted
    const HL &high_level       = stp_high_level;
    const Navigator &navigator = rrt_navigator;

    std::vector<std::unique_ptr<Intent>> assignedIntents =
        high_level.getIntentAssignment(world);

    std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
        navigator.getAssignedPrimitives(world, assignedIntents);

    return assignedPrimitives;
}
