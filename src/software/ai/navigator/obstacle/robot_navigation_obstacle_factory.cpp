#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

#include "software/ai/navigator/obstacle/const_velocity_obstacle.hpp"
#include "software/ai/navigator/obstacle/trajectory_obstacle.hpp"

RobotNavigationObstacleFactory::RobotNavigationObstacleFactory(
    TbotsProto::RobotNavigationObstacleConfig config)
    : config(config),
      robot_radius_expansion_amount(config.robot_obstacle_inflation_factor() *
                                    ROBOT_MAX_RADIUS_METERS)
{
}

std::vector<ObstaclePtr>
RobotNavigationObstacleFactory::createObstaclesFromMotionConstraint(
    const TbotsProto::MotionConstraint &motion_constraint,
    const WorldPtr &world_ptr) const
{
    std::vector<ObstaclePtr> obstacles;
    const Field &field = world_ptr->field();

    switch (motion_constraint)
    {
        case TbotsProto::MotionConstraint::CENTER_CIRCLE:
            obstacles.push_back(
                createFromShape(Circle(field.centerPoint(), field.centerCircleRadius())));
            break;
        case TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
        {
            obstacles.push_back(createFromFieldRectangle(
                field.enemyDefenseArea(), field.fieldLines(), field.fieldBoundary(),
                config.enemy_defense_area_additional_inflation_meters()));
        }
        break;
        case TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA:
            obstacles.push_back(createFromFieldRectangle(
                field.friendlyDefenseArea(), field.fieldLines(), field.fieldBoundary()));
            break;
        case TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA:
            obstacles.push_back(createFromFieldRectangle(
                field.enemyDefenseArea(), field.fieldLines(), field.fieldBoundary()));
            break;
        case TbotsProto::MotionConstraint::FRIENDLY_HALF:
            obstacles.push_back(createFromFieldRectangle(
                field.friendlyHalf(), field.fieldLines(), field.fieldBoundary()));
            break;
        case TbotsProto::MotionConstraint::ENEMY_HALF:
            obstacles.push_back(createFromFieldRectangle(
                field.enemyHalf(), field.fieldLines(), field.fieldBoundary()));
            break;
        case TbotsProto::MotionConstraint::ENEMY_HALF_WITHOUT_CENTRE_CIRCLE:
        {
            double radius                        = field.centerCircleRadius();
            Polygon centre_circle_and_enemy_half = Polygon(
                {Point(-robot_radius_expansion_amount,
                       field.fieldBoundary().yLength() / 2),
                 Point(-robot_radius_expansion_amount, radius), Point(0, radius),
                 Point(radius * std::cos(M_PI / 4), radius * std::sin(M_PI / 4)),
                 Point(radius, 0),
                 Point(radius * std::cos(M_PI / 4), -radius * std::sin(M_PI / 4)),
                 Point(0, -radius), Point(-robot_radius_expansion_amount, -radius),
                 Point(-robot_radius_expansion_amount,
                       -field.fieldBoundary().yLength() / 2),
                 field.fieldBoundary().posXNegYCorner(),
                 field.fieldBoundary().posXPosYCorner()});
            obstacles.push_back(
                std::make_shared<GeomObstacle<Polygon>>(centre_circle_and_enemy_half));
            break;
        }
        case TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE:
        {
            Rectangle field_walls    = field.fieldBoundary();
            Rectangle playable_field = field.fieldLines();
            // put each boundary zone as an obstacle
            Rectangle upper_boundary =
                Rectangle(field_walls.posXNegYCorner(),
                          {playable_field.xMax(), field_walls.yMax()});
            Rectangle left_boundary =
                Rectangle(field_walls.posXNegYCorner(),
                          {field_walls.xMin(), playable_field.yMin()});
            Rectangle right_boundary =
                Rectangle({field_walls.xMax(), playable_field.yMax()},
                          field_walls.negXPosYCorner());
            Rectangle lower_boundary =
                Rectangle({playable_field.xMin(), field_walls.yMin()},
                          field_walls.negXPosYCorner());
            obstacles.push_back(createFromShape(upper_boundary));
            obstacles.push_back(createFromShape(left_boundary));
            obstacles.push_back(createFromShape(right_boundary));
            obstacles.push_back(createFromShape(lower_boundary));
            break;
        }
        case TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL:;
            // 0.5 represents half a metre radius
            obstacles.push_back(
                createFromShape(Circle(world_ptr->ball().position(), 0.5)));
            break;
        case TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE:;
            if (world_ptr->gameState().getBallPlacementPoint().has_value())
            {
                obstacles.push_back(createFromBallPlacement(
                    world_ptr->gameState().getBallPlacementPoint().value(),
                    world_ptr->ball().position()));
            }
            else
            {
                obstacles.push_back(
                    createFromShape(Circle(world_ptr->ball().position(), 0.5)));
            }
            break;
        case TbotsProto::MotionConstraint::FRIENDLY_GOAL:
        {
            const Rectangle &friendly_goal = field.friendlyGoal();

            // Reduce the size of the goal obstacle slightly to avoid the goalie
            // appear to be inside the obstacle if it is touching one of the goal
            // walls (e.g. due to noisy vision).
            const double goal_obstacle_radius = ROBOT_MAX_RADIUS_METERS - 0.01;

            // Top goal post
            obstacles.push_back(std::make_shared<GeomObstacle<Stadium>>(Stadium(
                Segment(friendly_goal.posXPosYCorner(), friendly_goal.negXPosYCorner()),
                goal_obstacle_radius)));
            // Bottom goal post
            obstacles.push_back(std::make_shared<GeomObstacle<Stadium>>(Stadium(
                Segment(friendly_goal.posXNegYCorner(), friendly_goal.negXNegYCorner()),
                goal_obstacle_radius)));
            // Left goal wall
            obstacles.push_back(std::make_shared<GeomObstacle<Stadium>>(Stadium(
                Segment(friendly_goal.negXPosYCorner(), friendly_goal.negXNegYCorner()),
                goal_obstacle_radius)));
            break;
        }
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MIN_SENTINEL_DO_NOT_USE_:;
            break;
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MAX_SENTINEL_DO_NOT_USE_:;
            break;
    }

    return obstacles;
}

std::vector<ObstaclePtr>
RobotNavigationObstacleFactory::createObstaclesFromMotionConstraints(
    const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const WorldPtr &world_ptr) const
{
    std::vector<ObstaclePtr> obstacles;
    for (auto motion_constraint : motion_constraints)
    {
        auto new_obstacles =
            createObstaclesFromMotionConstraint(motion_constraint, world_ptr);
        obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    }

    return obstacles;
}

ObstaclePtr RobotNavigationObstacleFactory::createFromBallPosition(
    const Point &ball_position) const
{
    return createFromShape(Circle(ball_position, BALL_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createStadiumEnemyRobotObstacle(
    const Robot &enemy_robot) const
{
    Vector enemy_robot_velocity = enemy_robot.velocity();
    // Only generate a stadium obstacle if the robot is moving to avoid twitching
    // obstacles due to noisy velocity data
    if (enemy_robot_velocity.length() <
        config.dynamic_enemy_robot_obstacle_min_speed_mps())
    {
        return createStaticObstacleFromRobotPosition(enemy_robot.position());
    }

    return createFromShape(Stadium(
        enemy_robot.position(),
        enemy_robot.position() +
            enemy_robot_velocity * config.dynamic_enemy_robot_obstacle_horizon_sec(),
        ROBOT_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createConstVelocityEnemyRobotObstacle(
    const Robot &enemy_robot) const
{
    Vector enemy_robot_velocity = enemy_robot.velocity();
    // Only generate a const velocity obstacle if the robot is moving to avoid twitching
    // obstacles due to noisy velocity data
    if (enemy_robot_velocity.length() <
        config.dynamic_enemy_robot_obstacle_min_speed_mps())
    {
        return createStaticObstacleFromRobotPosition(enemy_robot.position());
    }

    return createCircleWithConstVelocity(
        Circle(enemy_robot.position(), ROBOT_MAX_RADIUS_METERS), enemy_robot.velocity());
}

ObstaclePtr RobotNavigationObstacleFactory::createStaticObstacleFromRobotPosition(
    const Point &robot_position) const
{
    return createFromShape(Circle(robot_position, ROBOT_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromMovingRobot(
    const Robot &robot, const TrajectoryPath &traj) const
{
    return createCircleWithTrajectory(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS),
                                      traj);
}

ObstaclePtr RobotNavigationObstacleFactory::createCircleWithTrajectory(
    const Circle &circle, const TrajectoryPath &traj) const
{
    return std::make_shared<TrajectoryObstacle<Circle>>(
        Circle(circle.origin(), circle.radius() + robot_radius_expansion_amount), traj);
}

ObstaclePtr RobotNavigationObstacleFactory::createCircleWithConstVelocity(
    const Circle &circle, const Vector &velocity) const
{
    return std::make_shared<ConstVelocityObstacle<Circle>>(
        Circle(circle.origin(), circle.radius() + robot_radius_expansion_amount),
        velocity, config.dynamic_enemy_robot_obstacle_horizon_sec());
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(const Circle &circle) const
{
    return std::make_shared<GeomObstacle<Circle>>(
        Circle(circle.origin(), circle.radius() + robot_radius_expansion_amount));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(const Polygon &polygon) const
{
    return std::make_shared<GeomObstacle<Polygon>>(
        polygon.expand(robot_radius_expansion_amount));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(
    const Rectangle &rectangle) const
{
    return std::make_shared<GeomObstacle<Rectangle>>(
        rectangle.expand(robot_radius_expansion_amount));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(const Stadium &stadium) const
{
    return std::make_shared<GeomObstacle<Stadium>>(
        Stadium(stadium.segment(), stadium.radius() + robot_radius_expansion_amount));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromFieldRectangle(
    const Rectangle &field_rectangle, const Rectangle &field_lines,
    const Rectangle &field_boundary, double additional_expansion_amount) const
{
    double xMin             = field_rectangle.xMin();
    double xMax             = field_rectangle.xMax();
    double yMin             = field_rectangle.yMin();
    double yMax             = field_rectangle.yMax();
    double expansion_amount = robot_radius_expansion_amount + additional_expansion_amount;

    xMin =
        (xMin == field_lines.xMin()) ? field_boundary.xMin() : (xMin - expansion_amount);
    xMax =
        (xMax == field_lines.xMax()) ? field_boundary.xMax() : (xMax + expansion_amount);
    yMin =
        (yMin == field_lines.yMin()) ? field_boundary.yMin() : (yMin - expansion_amount);
    yMax =
        (yMax == field_lines.yMax()) ? field_boundary.yMax() : (yMax + expansion_amount);

    return std::make_shared<GeomObstacle<Rectangle>>(
        Rectangle(Point(xMin, yMin), Point(xMax, yMax)));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromBallPlacement(
    const Point &placement_point, const Point &ball_point) const
{
    return createFromShape(Stadium(Segment(ball_point, placement_point), 0.5));
}
