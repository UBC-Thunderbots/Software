#include "software/ai/navigator/util.h"

#include <g3log/g3log.hpp>

#include "software/geom/util.h"
#include "software/new_geom/point.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).normalize().project((p3 - p2).normalize()).length();
}

std::vector<Obstacle> getObstaclesFromMotionConstraints(
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
                    getObstaclesFromTeam(world.enemyTeam());
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
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::ENEMY_DEFENSE_AREA:
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
            case MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
                rectangle = world.field().enemyDefenseArea();
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                         ->RobotObstacleInflationFactor()
                                         ->value() *
                                     ROBOT_MAX_RADIUS_METERS +
                                 0.3);  // 0.3 is by definition what inflated means
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::CENTER_CIRCLE:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.field().centerPoint(), world.field().centerCircleRadius(),
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case MotionConstraint::HALF_METER_AROUND_BALL:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.ball().position(), 0.5,  // 0.5 represents half a metre radius
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case MotionConstraint::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case MotionConstraint::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Vector(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
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
            MovePrimitive(robot_id, point, point.toVector().orientation(), final_speed,
                          enable_dribbler, MoveType::NORMAL, autokick);
        movePrimitives.emplace_back(movePrimitive);
    }

    return movePrimitives;
}

double getPointTrespass(const Point &p1, const Point &p2, double trespass_threshold)
{
    double dist_trespass = trespass_threshold - (p1 - p2).length();

    if (dist_trespass < 0)
    {
        dist_trespass = 0;
    }

    return dist_trespass;
}
