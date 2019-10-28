#pragma once
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/util/parameter/dynamic_parameters.h"

/**
 * ObstaclePathManager uses static_obstacles to arbitrate between paths to
 * implement the PathManager interface.
 */

class ObstaclePathManager : public PathManager
{
   public:
    const std::map<RobotId, Path> getManagedPaths(
        const std::map<RobotId, PathObjective> &objectives,
        const Rectangle &navigable_area,
        const std::vector<Obstacle> &static_obstacles) override;

    explicit ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner);

   private:
    /**
     * Creates obstacles around the start of objectives
     * except for current_index
     *
     * @param objectives objectives to make obstacles
     * @param current_index map key skip objective
     * @param inflation_factor how much to inflate obstacle
     *
     * @return list of obstacles that around other objectives' starts
     */
    const std::vector<Obstacle> getObstaclesAroundStartOfOtherObjectives(
        const std::map<RobotId, PathObjective> &objectives, size_t current_index,
        double inflation_factor);

    // Path planner used to get paths
    std::unique_ptr<PathPlanner> path_planner;
};
