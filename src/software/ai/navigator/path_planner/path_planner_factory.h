#pragma once
#include <span>
#include <typeinfo>

#include "software/ai/navigator/path_planner/path_planner.h"

class PathPlannerFactory
{
   public:
    using PathPlannerConstructor = std::function<std::unique_ptr<PathPlanner>()>;
    static const std::vector<
        std::pair<std::string, PathPlannerFactory::PathPlannerConstructor>>&
    getPathPlannerConstructors();

   protected:
    static void registerPathPlanner(std::string name, PathPlannerConstructor constructor);
    static std::vector<std::pair<std::string, PathPlannerFactory::PathPlannerConstructor>>
        path_planner_registry;
};

template <typename T>
class TPathPlannerFactory : public PathPlannerFactory
{
    static_assert(std::is_base_of<PathPlanner, T>::value,
                  "T must be a derived class of PathPlanner!");

   public:
    TPathPlannerFactory()
    {
        PathPlannerFactory::registerPathPlanner(typeid(T).name(),
                                                []() { return std::make_unique<T>(); });
    }
};