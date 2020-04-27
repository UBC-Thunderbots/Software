#include "software/ai/navigator/obstacle/obstacle_factory.h"

ObstacleFactory::ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config)
    : config(config)
{
}

std::vector<Obstacle> ObstacleFactory::getObstaclesFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world)
{
    double shape_expansion_amount =
        config->RobotObstacleInflationFactor()->value() * ROBOT_MAX_RADIUS_METERS;
    std::vector<Obstacle> obstacles;
    Rectangle rectangle({0, 0}, {1, 1});
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
                rectangle.expand(shape_expansion_amount);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::ENEMY_DEFENSE_AREA:
                // We extend the enemy defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().enemyDefenseArea().negXPosYCorner(),
                    Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
                rectangle.expand(shape_expansion_amount);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
                rectangle = world.field().enemyDefenseArea();
                // 0.3 is by definition what inflated means
                rectangle.expand(shape_expansion_amount + 0.3);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::CENTER_CIRCLE:
                obstacles.push_back(Obstacle(
                    Circle(world.field().centerPoint(),
                           world.field().centerCircleRadius() + shape_expansion_amount)));
                break;
            case MotionConstraint::HALF_METER_AROUND_BALL:
                // 0.5 represents half a metre radius
                obstacles.push_back(Obstacle(
                    Circle(world.ball().position(), 0.5 + shape_expansion_amount)));
                break;
            case MotionConstraint::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(shape_expansion_amount);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(shape_expansion_amount);
                obstacles.push_back(Obstacle(rectangle));
                break;
        }
    }

    return obstacles;
}

Obstacle ObstacleFactory::getVelocityObstacleFromRobot(const Robot &robot)
{
    double radius_cushion_scaling   = config->SpeedScalingFactor()->value();
    double velocity_cushion_scaling = config->RobotObstacleInflationFactor()->value();

    // radius cushion for a hexagonal approximation of a robot
    double radius_cushion =
        ROBOT_MAX_RADIUS_METERS * radius_cushion_scaling * 4.0 / std::sqrt(3);

    // vector in the direction of the velocity and with the scaled size of the
    // velocity
    Vector velocity_cushion_vector =
        robot.velocity().normalize(robot.velocity().length() * velocity_cushion_scaling +
                                   2 * ROBOT_MAX_RADIUS_METERS * radius_cushion_scaling);


    if (velocity_cushion_vector.length() > radius_cushion)
    {
        Vector velocity_direction_norm_radius =
            velocity_cushion_vector.normalize(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             robot.position() + velocity_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::fromDegrees(210)),
             // right side of robot
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::threeQuarter()),
             // right side velocity cushions
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::threeQuarter()) +
                 velocity_cushion_vector,
             // left side velocity cushions
             robot.position() + velocity_direction_norm_radius.rotate(Angle::quarter()) +
                 velocity_cushion_vector}));
    }
    else
    {
        // force the robot to face in +x direction
        Vector facing_direction_norm_radius = Vector(1, 0).normalize(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             robot.position() + facing_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::fromDegrees(210)),
             // right side of robot
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::threeQuarter()),
             // front right velocity cushions
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::fromDegrees(330)),
             // front left velocity cushions
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::fromDegrees(30))}));
    }
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
