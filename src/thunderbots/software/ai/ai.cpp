#include "ai/ai.h"

#include <chrono>

#include "ai/hl/stp/play/halt_play.h"
#include "ai/hl/stp/stp.h"
#include "ai/navigator/path_planning_navigator/path_planning_navigator.h"
#include "ai/navigator/placeholder_navigator/placeholder_navigator.h"

AI::AI()
    : navigator(std::make_unique<PathPlanningNavigator>()),
      // We use the current time in nanoseconds to initialize STP with a "random" seed
      high_level(std::make_unique<STP>(
          []() { return std::make_unique<HaltPlay>(); },
          std::chrono::system_clock::now().time_since_epoch().count()))
{
}

std::vector<std::unique_ptr<Primitive>> AI::getPrimitives(const World &world) const
{
    std::vector<std::unique_ptr<Intent>> assignedIntents = high_level->getIntents(world);

    std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
        navigator->getAssignedPrimitives(world, assignedIntents);

    return assignedPrimitives;
}

PlayInfo AI::getPlayInfo() const
{
    return high_level->getPlayInfo();
}
