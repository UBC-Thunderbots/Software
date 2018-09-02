#include "RobotObstacle.h"
#include "shared/constants.h"

using namespace DynamicParameters::Navigator;

RobotObstacle::RobotObstacle(Robot& robot, double avoid_dist)
{
    boundary = Circle(robot.position(), avoid_dist + ROBOT_MAX_RADIUS);
    velocity = Seg(robot.position(), robot.position() + robot.velocity() * collision_avoid_velocity_scale.value());
}

double RobotObstacle::getViolation(const Point& point)
{
    // Check if distance between p and center of boundary is less than the radius;
    // if so then we have a violation.
    double dist = (point - boundary.origin).len();
    return std::max(0.0, boundary.radius - dist);
}

Point RobotObstacle::getNearestValidPoint(const Point& point)
{
    if (getViolation(point) > 0.0)
    {
        return boundary.origin + (point - boundary.origin).norm(boundary.radius);
    }
    return point;
}

bool RobotObstacle::willCollide(Robot& robot)
{
    RobotObstacle other(robot, default_avoid_dist.value());
    return dist(velocity, other.velocity) < default_avoid_dist.value();
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
