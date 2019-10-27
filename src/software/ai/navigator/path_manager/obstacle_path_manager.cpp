#include "software/ai/navigator/path_manager/obstacle_path_manager.h"

/**
 * This file contains the implementation of the obstacle path manager
 * which uses obstacles to arbitrate between paths
 * ret a path consisting of only the start and destination
 * points.
 */


ObstaclePathManager::ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner)
    : path_planner(std::move(path_planner))
{
}

std::vector<Path> ObstaclePathManager::getManagedPaths(
    std::vector<PathObjective> objectives, const Rectangle &navigable_area,
    const std::vector<Obstacle> &static_obstacles)
{
    std::vector<Path> retval;

    std::vector<Obstacle>
        current_velocity_obstacles;  // velocity obstacles used to avoid collisions

    // cache dynamic params
    double robot_obstacle_inflation =
        Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor.value();
    double velocity_obstacle_inflation =
        Util::DynamicParameters::Navigator::velocity_obstacle_inflation_factor.value();

    for (size_t i = 0; i < objectives.size(); i++)
    {
        PathObjective current_objective = objectives[i];

        // find path with relevant obstacles
        std::vector<Obstacle> path_obstacles =
            getObstaclesFromOtherObjectives(objectives, i, robot_obstacle_inflation);
        path_obstacles.insert(path_obstacles.begin(), current_velocity_obstacles.begin(),
                              current_velocity_obstacles.end());
        Path path = path_planner->findPath(current_objective.start, current_objective.end,
                                           navigable_area, path_obstacles);

        // store path in retval
        retval.push_back(path);
        if (path)
        {
            // store velocity obstacle for current path
            std::vector<Point> path_points = path->getKnots();
            current_velocity_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    current_objective.start, path_points[1],
                    current_objective.current_velocity, robot_obstacle_inflation,
                    velocity_obstacle_inflation));
        }
    }

    return retval;
}

std::vector<Obstacle> ObstaclePathManager::getObstaclesFromOtherObjectives(
    std::vector<PathObjective> objectives, size_t current_index, double inflation_factor)
{
    std::vector<Obstacle> obstacles;
    for (size_t i = 0; i < objectives.size(); i++)
    {
        if (i != current_index)
        {
            obstacles.push_back(Obstacle::createCircleObstacle(
                objectives[i].start, ROBOT_MAX_RADIUS_METERS, inflation_factor));
        }
    }
    return obstacles;
}
