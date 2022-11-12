#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

RobotNavigationObstacleFactory::RobotNavigationObstacleFactory(
    TbotsProto::RobotNavigationObstacleConfig config)
    : config(config),
      robot_radius_expansion_amount(config.robot_obstacle_inflation_factor() *
                                    ROBOT_MAX_RADIUS_METERS)
{
}

std::vector<ObstaclePtr> RobotNavigationObstacleFactory::createFromMotionConstraint(
    const TbotsProto::MotionConstraint motion_constraint, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;
    auto static_obstacles =
        createStaticObstaclesFromMotionConstraint(motion_constraint, world.field());
    obstacles.insert(obstacles.end(), static_obstacles.begin(), static_obstacles.end());

    auto dynamic_obstacles =
        createDynamicObstaclesFromMotionConstraint(motion_constraint, world);
    obstacles.insert(obstacles.end(), dynamic_obstacles.begin(), dynamic_obstacles.end());

    CHECK(dynamic_obstacles.empty() || static_obstacles.empty())
        << "Motion constraint with value " << static_cast<int>(motion_constraint)
        << " has both dynamic and static obstacles." << std::endl;

    return obstacles;
}

std::vector<ObstaclePtr>
RobotNavigationObstacleFactory::createStaticObstaclesFromMotionConstraint(
    const TbotsProto::MotionConstraint &motion_constraint, const Field &field) const
{
    std::vector<ObstaclePtr> obstacles;

    switch (motion_constraint)
    {
        case TbotsProto::MotionConstraint::CENTER_CIRCLE:
            obstacles.push_back(
                createFromShape(Circle(field.centerPoint(), field.centerCircleRadius())));
            break;
        case TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
        {
            obstacles.push_back(createFromFieldRectangle(field.enemyDefenseArea(),
                                                         field.fieldLines(),
                                                         field.fieldBoundary(), 0.3));
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
            // HALF_METER_AROUND_BALL is not handled by this obstacle factory since it's a
            // dynamic obstacle
            break;
        case TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE:;
            // AVOID_BALL_PLACEMENT_INTERFERENCE is not handled by this obstacle factory
            // since it's a dynamic obstacle
            break;
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MIN_SENTINEL_DO_NOT_USE_:;
            break;
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MAX_SENTINEL_DO_NOT_USE_:;
            break;
    }

    return obstacles;
}

std::vector<ObstaclePtr>
RobotNavigationObstacleFactory::createDynamicObstaclesFromMotionConstraint(
    const TbotsProto::MotionConstraint &motion_constraint, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;

    switch (motion_constraint)
    {
        case TbotsProto::MotionConstraint::CENTER_CIRCLE:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::FRIENDLY_HALF:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::ENEMY_HALF:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE:
            // not handled by this obstacle factory since it's a static obstacle
            break;
        case TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL:;
            // 0.5 represents half a metre radius
            obstacles.push_back(createFromShape(Circle(world.ball().position(), 0.5)));
            break;
        case TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE:;
            if (world.gameState().getBallPlacementPoint().has_value())
            {
                obstacles.push_back(createFromBallPlacement(
                    world.gameState().getBallPlacementPoint().value(),
                    world.ball().position()));
            }
            else
            {
                obstacles.push_back(
                    createFromShape(Circle(world.ball().position(), 0.5)));
            }
            break;
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MIN_SENTINEL_DO_NOT_USE_:;
            break;
        case TbotsProto::MotionConstraint::MotionConstraint_INT_MAX_SENTINEL_DO_NOT_USE_:;
            break;
    }

    return obstacles;
}

std::vector<ObstaclePtr>
RobotNavigationObstacleFactory::createStaticObstaclesFromMotionConstraints(
    const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const Field &field) const
{
    std::vector<ObstaclePtr> obstacles;
    for (auto motion_constraint : motion_constraints)
    {
        auto new_obstacles =
            createStaticObstaclesFromMotionConstraint(motion_constraint, field);
        obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    }

    return obstacles;
}

ObstaclePtr RobotNavigationObstacleFactory::createFromBallPosition(
    const Point &ball_position) const
{
    return createFromShape(Circle(ball_position, BALL_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromRobotPosition(
    const Point &robot_position) const
{
    return createFromShape(Circle(robot_position, ROBOT_MAX_RADIUS_METERS));
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
    return createFromShape(
        Polygon::fromSegment(Segment(ball_point, placement_point), 0.5));
}
