#pragma once
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_manager/path_manager.h"

/**
 * VelocityObstaclePathManager uses obstacles to arbitrate between paths.
 * The added obstacles to cause the path planner to generate paths that avoid potential
 * collisions. This approach implicitly uses the idea of [Minkowski
 * space](https://en.wikipedia.org/wiki/Minkowski_space), but where we assume that a robot
 * will occupy all the positions along the path for the next time step.
 */

class VelocityObstaclePathManager : public PathManager
{
   public:
    const std::map<RobotId, std::optional<Path>> getManagedPaths(
        const std::unordered_set<PathObjective>& objectives,
        const Rectangle& navigable_area) override;
    const std::vector<ObstaclePtr> getObstacles(void) const override;

    explicit VelocityObstaclePathManager(
        std::unique_ptr<PathPlanner> path_planner,
        RobotNavigationObstacleFactory robot_navigation_obstacle_factory);


   private:
    /**
     * Creates obstacles around the start of objectives
     * except for current_index
     *
     * @param objectives objectives to make obstacles
     * @param current_objective objective to skip
     *
     * @return list of obstacles that around other objectives' starts
     */
    const std::vector<ObstaclePtr> getObstaclesAroundStartOfOtherObjectives(
        const std::unordered_set<PathObjective>& objectives,
        const PathObjective& current_objective);

    std::unique_ptr<PathPlanner> path_planner;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    std::vector<ObstaclePtr> path_planning_obstacles;
};
