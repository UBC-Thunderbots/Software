#include "software/ai/navigator/path_planning_navigator/obstacle/robot_obstacle.h"

#include "shared/constants.h"

RobotObstacle::RobotObstacle(const Robot& robot, double avoid_dist)
{
    boundary = Circle(robot.position(), avoid_dist + ROBOT_MAX_RADIUS_METERS);
    velocity = Segment(
        robot.position(),
        robot.position() + robot.velocity() * Util::DynamicParameters->getNavigatorConfig()->CollisionAvoidVelocityScale()->value());
}

double RobotObstacle::getViolationDistance(const Point& point)
{
    // Check if distance between p and center of boundary is less than the radius
    // if so then we have a violation.
    double dist1 = (point - boundary.getOrigin()).len();
    return std::max(0.0, boundary.getRadius() - dist1);
}

Point RobotObstacle::getNearestValidPoint(const Point& point)
{
    if (getViolationDistance(point) > 0.0)
    {
        return boundary.getOrigin() +
               (point - boundary.getOrigin()).norm(boundary.getRadius());
    }
    return point;
}

bool RobotObstacle::willCollide(const Robot& robot)
{
    RobotObstacle other = RobotObstacle(robot, Util::DynamicParameters->getNavigatorConfig()->CollisionAvoidVelocityScale()->value());
    return dist(velocity, other.velocity) < Util::DynamicParameters->getNavigatorConfig()->CollisionAvoidVelocityScale()->value();
}

std::vector<RobotObstacle> generate_friendly_obstacles(const Team& friendly_team,
                                                       double avoid_dist)
{
    std::vector<RobotObstacle> obst;
    for (Robot r : friendly_team.getAllRobots())
    {
        obst.push_back(RobotObstacle(r, avoid_dist));
    }

    return obst;
}

std::vector<RobotObstacle> generate_enemy_obstacles(const Team& enemy_team,
                                                    double avoid_dist)
{
    // No difference in implementation for now.
    return generate_friendly_obstacles(enemy_team, avoid_dist);
}
