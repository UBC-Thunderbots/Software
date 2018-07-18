#include "rrt.h"

RRTNav::RRTNav()
{
}

std::map<unsigned int, Primitive> RRTNav::getAssignedPrimitives(
    const std::vector<std::pair<unsigned int, Intent>> &assignedIntents,
    const World &world)
{
    return std::map<unsigned int, Primitive>();
}