#include "ai.h"

AI::AI() : rrt_navigator(), stp_high_level() {

}

std::vector<std::unique_ptr<Primitive>> AI::getPrimitives() const {
    // Use generic variables with the types of the Abstract base clases
    // so we can only operate in terms of the public interfaces of these modules
    //
    // These can't be declared in the header because the default copy
    // constructor for the class will be automatically deleted
    const HL &high_level = stp_high_level;
    const Navigator &navigator = rrt_navigator;

    std::vector<std::unique_ptr<Intent>> assignedIntents =
            high_level.getIntentAssignment(world);

    std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
            navigator.getAssignedPrimitives(world, assignedIntents);

    return assignedPrimitives;
}
