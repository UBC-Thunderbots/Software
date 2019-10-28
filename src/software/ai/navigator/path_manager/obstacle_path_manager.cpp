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

const std::map<int, Path> ObstaclePathManager::getManagedPaths(
    const std::map<int, PathObjective> &objectives, const Rectangle &navigable_area,
    const std::vector<Obstacle> &static_obstacles)
{
    std::map<int, Path> retval;

    std::vector<Obstacle>
        current_velocity_obstacles;  // velocity obstacles used to avoid collisions

    // cache dynamic params
    double robot_obstacle_inflation =
        Util::DynamicParameters->getNavigatorConfig()->RobotObstacleInflationFactor()->value();
    double velocity_obstacle_inflation =
        Util::DynamicParameters->getNavigatorConfig()->VelocityObstacleInflationFactor()->value();

    for (auto const &current_objective : objectives)
    {
        // find path with relevant obstacles
        std::vector<Obstacle> path_obstacles = getObstaclesFromOtherObjectives(
            objectives, current_objective.first, robot_obstacle_inflation);
        path_obstacles.insert(path_obstacles.begin(), current_velocity_obstacles.begin(),
                              current_velocity_obstacles.end());
        path_obstacles.insert(path_obstacles.begin(),
                              current_objective.second.avoid_area_obstacles.begin(),
                              current_objective.second.avoid_area_obstacles.end());
        Path path = path_planner->findPath(current_objective.second.start,
                                           current_objective.second.end, navigable_area,
                                           path_obstacles);

        // store path in retval
        retval.insert({current_objective.first, path});

        // store velocity obstacle for current path
        if (path)
        {
            std::vector<Point> path_points = path->getKnots();
            current_velocity_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    current_objective.second.start, path_points[1],
                    current_objective.second.current_velocity, robot_obstacle_inflation,
                    velocity_obstacle_inflation));
        }
    }

    return retval;
}

const std::vector<Obstacle> ObstaclePathManager::getObstaclesFromOtherObjectives(
    const std::map<int, PathObjective> &objectives, size_t current_index,
    double inflation_factor)
{
    std::vector<Obstacle> obstacles;
    for (auto const &obj : objectives)
    {
        if (obj.first != current_index)
        {
            obstacles.push_back(Obstacle::createCircleObstacle(
                obj.second.start, ROBOT_MAX_RADIUS_METERS, inflation_factor));
        }
    }
    return obstacles;
}
