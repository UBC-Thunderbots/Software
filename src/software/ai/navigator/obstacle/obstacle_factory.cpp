#include "software/ai/navigator/obstacle/obstacle_factory.h"

ObstacleFactory::ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config)
    : config(config),
      shape_expansion_amount(config->RobotObstacleInflationFactor()->value() *
                             ROBOT_MAX_RADIUS_METERS)
{
}

std::vector<Obstacle> ObstacleFactory::createObstaclesFromMotionConstraint(
    const MotionConstraint &motion_constraint, const World &world)
{
    std::vector<Obstacle> obstacles;


    if (motion_constraint == MotionConstraint::ENEMY_ROBOTS_COLLISION)
    {
        std::vector<Obstacle> enemy_robot_obstacles =
            createVelocityObstaclesFromTeam(world.enemyTeam());
        obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                         enemy_robot_obstacles.end());
    }
    else if (motion_constraint == MotionConstraint::CENTER_CIRCLE)
    {
        obstacles.push_back(createObstacle(
            Circle(world.field().centerPoint(),
                   world.field().centerCircleRadius() + shape_expansion_amount)));
    }
    else if (motion_constraint == MotionConstraint::HALF_METER_AROUND_BALL)
    {
        // 0.5 represents half a metre radius
        obstacles.push_back(createObstacle(
            Circle(world.ball().position(), 0.5 + shape_expansion_amount)));
    }
    else if (motion_constraint == MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA)
    {
        Rectangle rectangle = world.field().enemyDefenseArea();
        // 0.3 is by definition what inflated means
        rectangle.expand(shape_expansion_amount + 0.3);
        obstacles.push_back(createObstacle(rectangle));
    }
    else
    {
        Rectangle rectangle({0, 0}, {1, 1});

        switch (motion_constraint)
        {
            case MotionConstraint::FRIENDLY_DEFENSE_AREA:
                // We extend the friendly defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().friendlyDefenseArea().posXPosYCorner(),
                    Point(-10, world.field().friendlyDefenseArea().posXNegYCorner().y()));
                break;
            case MotionConstraint::ENEMY_DEFENSE_AREA:
                // We extend the enemy defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().enemyDefenseArea().negXPosYCorner(),
                    Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
                break;
            case MotionConstraint::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                break;
            case MotionConstraint::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                break;
            default:
                // already handled in previous if/else
                break;
        }
        rectangle.expand(shape_expansion_amount);
        obstacles.push_back(createObstacle(rectangle));
    }
    return obstacles;
}

std::vector<Obstacle> ObstacleFactory::createObstaclesFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world)
{
    std::vector<Obstacle> obstacles;
    for (auto motion_constraint : motion_constraints)
    {
        auto new_obstacles =
            createObstaclesFromMotionConstraint(motion_constraint, world);
        obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    }

    return obstacles;
}

Obstacle ObstacleFactory::createVelocityObstacleFromRobot(const Robot &robot)
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
        // use hexagonal approximation for velocity obstacle
        Vector velocity_direction_norm_radius =
            velocity_cushion_vector.normalize(radius_cushion);
        return createObstacle(Polygon(
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
        return createObstacle(Circle(robot.position(), radius_cushion));
    }
}

Obstacle ObstacleFactory::createRobotObstacle(const Point &robot_position,
                                              const double radius_scaling)
{
    return createObstacle(
        Circle(robot_position,
               radius_scaling * (ROBOT_MAX_RADIUS_METERS + shape_expansion_amount)));
}

std::vector<Obstacle> ObstacleFactory::createVelocityObstaclesFromTeam(const Team &team)
{
    std::vector<Obstacle> obstacles;
    for (const auto &robot : team.getAllRobots())
    {
        obstacles.push_back(createVelocityObstacleFromRobot(robot));
    }
    return obstacles;
}

Obstacle ObstacleFactory::createBallObstacle(const Point &ball_position,
                                             double additional_radius_cushion_buffer)
{
    return createObstacle(Circle(ball_position, BALL_MAX_RADIUS_METERS +
                                                    additional_radius_cushion_buffer +
                                                    shape_expansion_amount));
}

Obstacle ObstacleFactory::createObstacle(const Circle &circle)
{
    return Obstacle(std::make_shared<CircleObstacle>(CircleObstacle(circle)));
}

Obstacle ObstacleFactory::createObstacle(const Polygon &polygon)
{
    return Obstacle(std::make_shared<PolygonObstacle>(PolygonObstacle(polygon)));
}
