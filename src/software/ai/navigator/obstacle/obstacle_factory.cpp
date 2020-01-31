#include "software/ai/navigator/obstacle/obstacle_factory.h"

ObstacleFactory::ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config)
    : config(config)
{
}

std::vector<Obstacle> ObstacleFactory::getObstaclesFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world)
{
    std::vector<Obstacle> obstacles;
    Rectangle rectangle({0, 0}, {0, 0});
    for (auto motion_constraint : motion_constraints)
    {
        switch (motion_constraint)
        {
            case MotionConstraint::ENEMY_ROBOTS_COLLISION:
            {
                std::vector<Obstacle> enemy_robot_obstacles =
                    getVelocityObstaclesFromTeam(world.enemyTeam());
                obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                                 enemy_robot_obstacles.end());
            }
            break;
            case MotionConstraint::FRIENDLY_DEFENSE_AREA:
                // We extend the friendly defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().friendlyDefenseArea().posXPosYCorner(),
                    Point(-10, world.field().friendlyDefenseArea().posXNegYCorner().y()));
                rectangle.expand(config->RobotObstacleInflationFactor()->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::ENEMY_DEFENSE_AREA:
                // We extend the enemy defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().enemyDefenseArea().negXPosYCorner(),
                    Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
                rectangle.expand(config->RobotObstacleInflationFactor()->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
                rectangle = world.field().enemyDefenseArea();
                rectangle.expand(config->RobotObstacleInflationFactor()->value() *
                                     ROBOT_MAX_RADIUS_METERS +
                                 0.3);  // 0.3 is by definition what inflated means
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::CENTER_CIRCLE:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.field().centerPoint(), world.field().centerCircleRadius(),
                    config->RobotObstacleInflationFactor()->value()));
                break;
            case MotionConstraint::HALF_METER_AROUND_BALL:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.ball().position(), 0.5,  // 0.5 represents half a metre radius
                    config->RobotObstacleInflationFactor()->value()));
                break;
            case MotionConstraint::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(config->RobotObstacleInflationFactor()->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(config->RobotObstacleInflationFactor()->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
        }
    }

    return obstacles;
}

Obstacle ObstacleFactory::getVelocityObstacleFromRobot(const Robot &robot)
{
    double speed_scaling_factor = config->SpeedScalingFactor()->value();
    double robot_scaling_factor = config->RobotObstacleInflationFactor()->value();

    return Obstacle::createRobotObstacleWithScalingParams(robot, speed_scaling_factor,
                                                          robot_scaling_factor);
}

std::vector<Obstacle> ObstacleFactory::getVelocityObstaclesFromTeam(const Team &team)
{
    std::vector<Obstacle> obstacles;
    for (const auto &robot : team.getAllRobots())
    {
        obstacles.push_back(getVelocityObstacleFromRobot(robot));
    }
    return obstacles;
}
