#include "software/ai/navigator/path_planner/path_planner_factory.h"

std::vector<std::pair<std::string, PathPlannerFactory::PathPlannerConstructor>>
    PathPlannerFactory::path_planner_registry;

const std::vector<std::pair<std::string, PathPlannerFactory::PathPlannerConstructor>>
    &PathPlannerFactory::getPathPlannerConstructors()
{
    return path_planner_registry;
}

void PathPlannerFactory::registerPathPlanner(
    std::string name, PathPlannerFactory::PathPlannerConstructor constructor)
{
    path_planner_registry.emplace_back(name, constructor);
}
