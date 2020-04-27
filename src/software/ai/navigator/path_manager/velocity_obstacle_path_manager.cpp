#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"

VelocityObstaclePathManager::VelocityObstaclePathManager(
    std::unique_ptr<PathPlanner> path_planner, ObstacleFactory obstacle_factory,
    std::shared_ptr<const VelocityObstaclePathManagerConfig> config)
    : path_planner(std::move(path_planner)),
      obstacle_factory(std::move(obstacle_factory)),
      config(config)
{
}

const std::map<RobotId, std::optional<Path>> VelocityObstaclePathManager::getManagedPaths(
    const std::unordered_set<PathObjective> &objectives, const Rectangle &navigable_area)
{
    std::map<RobotId, std::optional<Path>> managed_paths;

    // Velocity obstacles used to avoid collisions.
    // As we plan a path for each robot, a corresponding obstacle will be added
    // to this list so that paths planned later do not collide with the path we just
    // planned. Please see: https://en.wikipedia.org/wiki/Velocity_obstacle
    std::vector<Obstacle> current_velocity_obstacles;

    for (auto const &current_objective : objectives)
    {
        // find path with relevant obstacles
        std::vector<Obstacle> path_obstacles =
            getObstaclesAroundStartOfOtherObjectives(objectives, current_objective);
        path_obstacles.insert(path_obstacles.end(), current_velocity_obstacles.begin(),
                              current_velocity_obstacles.end());
        path_obstacles.insert(path_obstacles.end(), current_objective.obstacles.begin(),
                              current_objective.obstacles.end());
        auto path = path_planner->findPath(current_objective.start, current_objective.end,
                                           navigable_area, path_obstacles);

        // store path in managed_paths
        managed_paths.insert({current_objective.robot_id, path});

        // store velocity obstacle for current path
        if (path && path->size() >= 2)
        {
            // We want to avoid the start of every other path, assuming that
            // there is a robot moving along the path from the path's start
            std::vector<Point> path_points = path->getKnots();
            Vector initial_path_velocity =
                (path_points[1] - current_objective.start)
                    .normalize(current_objective.current_speed);
            Robot mock_path_robot(0, current_objective.start, initial_path_velocity,
                                  Angle::zero(), AngularVelocity::zero(),
                                  Timestamp::fromSeconds(0));
            current_velocity_obstacles.emplace_back(
                obstacle_factory.createVelocityObstacleFromRobot(mock_path_robot));
        }
    }

    return managed_paths;
}

const std::vector<Obstacle>
VelocityObstaclePathManager::getObstaclesAroundStartOfOtherObjectives(
    const std::unordered_set<PathObjective> &objectives,
    const PathObjective &current_objective)
{
    double inflation_factor = config->OtherPathObjectiveStartInflationFactor()->value();
    std::vector<Obstacle> obstacles;
    for (auto const &obj : objectives)
    {
        if (obj != current_objective)
        {
            obstacles.push_back(
                obstacle_factory.createRobotObstacle(obj.start, inflation_factor));
        }
    }
    return obstacles;
}
