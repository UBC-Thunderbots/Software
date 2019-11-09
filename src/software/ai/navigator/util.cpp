#include "software/ai/navigator/util.h"

#include <g3log/g3log.hpp>

#include "software/geom/point.h"
#include "software/geom/util.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).norm().project((p3 - p2).norm()).len();
}

std::vector<Obstacle> getObstaclesFromAvoidAreas(
    const std::vector<AvoidArea> &avoid_areas, World world)
{
    std::vector<Obstacle> obstacles;
    Rectangle rectangle({0, 0}, {0, 0});
    for (auto avoid_area : avoid_areas)
    {
        switch (avoid_area)
        {
            case AvoidArea::ENEMY_ROBOTS:
            {
                std::vector<Obstacle> enemy_robot_obstacles =
                    getObstaclesFromTeam(world.enemyTeam());
                obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                                 enemy_robot_obstacles.end());
            }
            break;
            case AvoidArea::FRIENDLY_DEFENSE_AREA:
                // We extend the friendly defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().friendlyDefenseArea().posXPosYCorner(),
                    Point(-10, world.field().friendlyDefenseArea().posXNegYCorner().y()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::ENEMY_DEFENSE_AREA:
                // We extend the enemy defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().enemyDefenseArea().negXPosYCorner(),
                    Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::INFLATED_ENEMY_DEFENSE_AREA:
                rectangle = world.field().enemyDefenseArea();
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                         ->RobotObstacleInflationFactor()
                                         ->value() *
                                     ROBOT_MAX_RADIUS_METERS +
                                 0.3);  // 0.3 is by definition what inflated means
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::CENTER_CIRCLE:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.field().centerPoint(), world.field().centerCircleRadius(),
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case AvoidArea::HALF_METER_AROUND_BALL:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.ball().position(), 0.5,  // 0.5 represents half a metre radius
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case AvoidArea::BALL:
                obstacles.push_back(
                    Obstacle::createCircularBallObstacle(world.ball(), 0.06));
                break;
            case AvoidArea::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Point(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Point(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            default:
                LOG(WARNING) << "Could not convert AvoidArea " << (int)avoid_area
                             << " to obstacle";
        }
    }

    return obstacles;
}

std::vector<Obstacle> getObstaclesFromTeam(const Team &team)
{
    double robot_inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                        ->RobotObstacleInflationFactor()
                                        ->value();
    double velocity_inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                           ->VelocityObstacleInflationFactor()
                                           ->value();
    std::vector<Obstacle> obstacles;
    for (auto &robot : team.getAllRobots())
    {
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(
            robot, robot_inflation_factor, velocity_inflation_factor);
        obstacles.push_back(o);
    }
    return obstacles;
}

std::vector<MovePrimitive> convertToMovePrimitives(unsigned int robot_id,
                                                   const std::vector<Point> &points,
                                                   DribblerEnable enable_dribbler,
                                                   AutokickType autokick)
{
    std::vector<MovePrimitive> movePrimitives;
    movePrimitives.reserve(points.size());

    for (unsigned index = 0; index < points.size(); index++)
    {
        const Point &point = points.at(index);

        double final_speed = 0;
        if (index < points.size() - 2)
        {
            const Point &next_point      = points.at(index + 1);
            const Point &next_next_point = points.at(index + 2);

            final_speed = calculateTransitionSpeedBetweenSegments(point, next_point,
                                                                  next_next_point, 0);
        }

        MovePrimitive movePrimitive =
            MovePrimitive(robot_id, point, point.orientation(), final_speed,
                          enable_dribbler, MoveType::NORMAL, autokick);
        movePrimitives.emplace_back(movePrimitive);
    }

    return movePrimitives;
}

double getPointTrespass(const Point &p1, const Point &p2, double trespass_threshold)
{
    double dist_trespass = trespass_threshold - (p1 - p2).len();

    if (dist_trespass < 0)
    {
        dist_trespass = 0;
    }

    return dist_trespass;
}
