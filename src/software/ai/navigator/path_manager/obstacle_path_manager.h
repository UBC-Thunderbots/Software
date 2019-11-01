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
    const std::map<PathObjective, std::optional<Path>> getManagedPaths(
        const std::set<PathObjective> &objectives,
        const Rectangle &navigable_area) override;

    explicit ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner);

   private:
    /**
     * Creates obstacles around the start of objectives
     * except for current_index
     *
     * @param objectives objectives to make obstacles
     * @param current_objective objective to skip
     * @param inflation_factor how much to inflate obstacle
     *
     * @return list of obstacles that around other objectives' starts
     */
    const std::vector<Obstacle> getObstaclesAroundStartOfOtherObjectives(
        const std::set<PathObjective> &objectives, const PathObjective &current_objective,
        double inflation_factor);

    // Path planner used to get paths
    std::unique_ptr<PathPlanner> path_planner;
};
