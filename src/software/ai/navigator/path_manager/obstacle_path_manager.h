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
    const std::map<int, Path> getManagedPaths(
        const std::map<int, PathObjective> &objectives, const Rectangle &navigable_area,
        const std::vector<Obstacle> &static_obstacles) override;

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
    const std::vector<Obstacle> getObstaclesFromOtherObjectives(
        const std::map<int, PathObjective> &objectives, size_t current_index,
        double inflation_factor);

    explicit ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner);

    // Path planner used to get paths
    std::unique_ptr<PathPlanner> path_planner;
};
