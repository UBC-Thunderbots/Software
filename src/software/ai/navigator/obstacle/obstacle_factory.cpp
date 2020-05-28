#include "software/ai/navigator/obstacle/obstacle_factory.h"

ObstacleFactory::ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config)
    : config(config),
      obstacle_expansion_amount(config->RobotObstacleInflationFactor()->value() *
                                ROBOT_MAX_RADIUS_METERS)
{
}

std::vector<ObstaclePtr> ObstacleFactory::createObstaclesFromMotionConstraint(
    const MotionConstraint &motion_constraint, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;

    switch (motion_constraint)
    {
        case MotionConstraint::ENEMY_ROBOTS_COLLISION:
        {
            std::vector<ObstaclePtr> enemy_robot_obstacles =
                createVelocityObstaclesFromTeam(world.enemyTeam());
            obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                             enemy_robot_obstacles.end());
        }
        break;
        case MotionConstraint::CENTER_CIRCLE:
            obstacles.push_back(expandForRobotSize(
                Circle(world.field().centerPoint(), world.field().centerCircleRadius())));
            break;
        case MotionConstraint::HALF_METER_AROUND_BALL:
            // 0.5 represents half a metre radius
            obstacles.push_back(expandForRobotSize(Circle(world.ball().position(), 0.5)));
            break;
        case MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
        {
            obstacles.push_back(expandThreeSidesForInflatedRobotSize(
                world.field().enemyDefenseAreaToBoundary(), TeamType::ENEMY));
        }
        break;
        case MotionConstraint::FRIENDLY_DEFENSE_AREA:
            obstacles.push_back(expandThreeSidesForRobotSize(
                world.field().friendlyDefenseAreaToBoundary(), TeamType::FRIENDLY));
            break;
        case MotionConstraint::ENEMY_DEFENSE_AREA:
            obstacles.push_back(expandThreeSidesForRobotSize(
                world.field().enemyDefenseAreaToBoundary(), TeamType::ENEMY));
            break;
        case MotionConstraint::FRIENDLY_HALF:
            obstacles.push_back(expandOneSideForRobotSize(
                world.field().friendlyHalfToBoundary(), TeamType::FRIENDLY));
            break;
        case MotionConstraint::ENEMY_HALF:
            obstacles.push_back(expandOneSideForRobotSize(
                world.field().enemyHalfToBoundary(), TeamType::ENEMY));
            break;
    }

    return obstacles;
}

std::vector<ObstaclePtr> ObstacleFactory::createObstaclesFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;
    for (auto motion_constraint : motion_constraints)
    {
        auto new_obstacles =
            createObstaclesFromMotionConstraint(motion_constraint, world);
        obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    }

    return obstacles;
}

ObstaclePtr ObstacleFactory::createVelocityObstacleFromRobot(const Robot &robot) const
{
    // radius of a hexagonal approximation of a robot
    double robot_hexagon_radius =
        (ROBOT_MAX_RADIUS_METERS + obstacle_expansion_amount) * 2.0 / std::sqrt(3);

    // vector in the direction of the velocity and proportional to the norm the velocity
    Vector inflated_velocity_vector = robot.velocity().normalize(
        robot.velocity().length() * config->SpeedScalingFactor()->value() +
        obstacle_expansion_amount);

    /* If the robot is travelling slower than a threshold, then a stationary robot
     * obstacle will be returned. If the robot is travelling faster than a threshold, then
     * the robot will be represented by a velocity obstacle, which is an irregular hexagon
     * like so:
     *
     *                        _____
     *                       /     \
     *                      /       \
     *       The robot >   +    R    +       <
     *       is at R       |         |       |
     *                     |         |       | The length of the velocity
     *                     |         |       | obstacle extension is
     *                     |         |       | proportional to the robot velocity
     *                     |         |       |
     *                     +---------+       <
     *                          |
     *                          |
     *                          V
     *                velocity of the robot
     */

    if (inflated_velocity_vector.length() > robot_hexagon_radius)
    {
        Vector velocity_norm_radius =
            inflated_velocity_vector.normalize(robot_hexagon_radius);
        return std::make_shared<GeomObstacle<Polygon>>(Polygon(
            {// left side of robot
             robot.position() + velocity_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             robot.position() + velocity_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             robot.position() + velocity_norm_radius.rotate(Angle::fromDegrees(210)),
             // right side of robot
             robot.position() + velocity_norm_radius.rotate(Angle::threeQuarter()),
             // right side of velocity obstacle extension
             robot.position() + velocity_norm_radius.rotate(Angle::threeQuarter()) +
                 inflated_velocity_vector,
             // left side of velocity obstacle extension
             robot.position() + velocity_norm_radius.rotate(Angle::quarter()) +
                 inflated_velocity_vector}));
    }
    else
    {
        return createRobotObstacle(robot.position());
    }
}

std::vector<ObstaclePtr> ObstacleFactory::createVelocityObstaclesFromTeam(
    const Team &team) const
{
    std::vector<ObstaclePtr> obstacles;
    for (const auto &robot : team.getAllRobots())
    {
        obstacles.push_back(createVelocityObstacleFromRobot(robot));
    }
    return obstacles;
}

ObstaclePtr ObstacleFactory::createBallObstacle(const Point &ball_position) const
{
    return expandForRobotSize(Circle(ball_position, BALL_MAX_RADIUS_METERS + 0.06));
}

ObstaclePtr ObstacleFactory::createRobotObstacle(const Point &robot_position) const
{
    return expandForRobotSize(Circle(robot_position, ROBOT_MAX_RADIUS_METERS));
}

ObstaclePtr ObstacleFactory::createObstacleFromRectangle(const Rectangle &rectangle) const
{
    return expandForRobotSize(Polygon(rectangle));
}

ObstaclePtr ObstacleFactory::expandForRobotSize(const Circle &circle) const
{
    return std::make_shared<GeomObstacle<Circle>>(
        Circle(circle.getOrigin(), circle.getRadius() + obstacle_expansion_amount));
}

ObstaclePtr ObstacleFactory::expandForRobotSize(const Polygon &polygon) const
{
    return std::make_shared<GeomObstacle<Polygon>>(
        polygon.expand(obstacle_expansion_amount * Vector(-1, 0))
            .expand(obstacle_expansion_amount * Vector(1, 0))
            .expand(obstacle_expansion_amount * Vector(0, -1))
            .expand(obstacle_expansion_amount * Vector(0, 1)));
}

ObstaclePtr ObstacleFactory::expandThreeSidesForRobotSize(const Rectangle &rectangle,
                                                          TeamType team_type) const
{
    return expandOneSideForRobotSize(
        rectangle.expand(obstacle_expansion_amount * Vector(0, -1))
            .expand(obstacle_expansion_amount * Vector(0, 1)),
        team_type);
}

ObstaclePtr ObstacleFactory::expandOneSideForRobotSize(const Rectangle &rectangle,
                                                       TeamType team_type) const
{
    if (team_type == TeamType::FRIENDLY)
    {
        return std::make_shared<GeomObstacle<Polygon>>(
            rectangle.expand(obstacle_expansion_amount * Vector(1, 0)));
    }
    else
    {
        return std::make_shared<GeomObstacle<Polygon>>(
            rectangle.expand(obstacle_expansion_amount * Vector(-1, 0)));
    }
}

ObstaclePtr ObstacleFactory::expandThreeSidesForInflatedRobotSize(
    const Rectangle &rectangle, TeamType team_type) const
{
    double inflated_expansion_amount = obstacle_expansion_amount + 0.3;
    Rectangle vert_exp_rect = rectangle.expand(inflated_expansion_amount * Vector(0, -1))
                                  .expand(inflated_expansion_amount * Vector(0, 1));
    if (team_type == TeamType::FRIENDLY)
    {
        return std::make_shared<GeomObstacle<Polygon>>(
            vert_exp_rect.expand({inflated_expansion_amount, 0}));
    }
    else
    {
        return std::make_shared<GeomObstacle<Polygon>>(
            vert_exp_rect.expand({-inflated_expansion_amount, 0}));
    }
}
