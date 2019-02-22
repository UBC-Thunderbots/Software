#include "astar.h"

#include <exception>

#include "ai/intent/move_intent.h"
#include "ai/navigator/RobotObstacle.h"
#include "ai/primitive/move_primitive.h"

AStarNav::AStarNav() {}

std::vector<std::unique_ptr<Primitive>> AStarNav::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents) const
{
    return std::vector<std::unique_ptr<Primitive>>();
}
