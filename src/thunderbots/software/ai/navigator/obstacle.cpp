#include "obstacle.h"
#include "shared/constants.h"

RobotObstacle::RobotObstacle(Robot& r, double avoid_dist)
{
    boundary = Circle(r.position(), avoid_dist + ROBOT_MAX_RADIUS);
    velocity = Seg(r.position(), r.position() + r.velocity() * DEFAULT_VELOCITY_SCALE);
}

double RobotObstacle::getViolation(Point& p)
{
    // Check if distance between p and center of boundary is less than the radius;
    // if so then we have a violation.
    double dist = (p - boundary.origin).len();
    return std::max(0.0, boundary.radius - dist);
}

Point RobotObstacle::getNearestValidPoint(Point& p)
{
    if (getViolation(p) > 0.0)
    {
        return boundary.origin + (p - boundary.origin).norm(boundary.radius);
    }
    return p;
}

bool RobotObstacle::willCollide(Robot& r)
{
    RobotObstacle other(r, DEFAULT_AVOID_DIST);
    return dist(velocity, other.velocity) < DEFAULT_AVOID_DIST;
}

std::vector<RobotObstacle> process_friendly_obstacles(const Team& friendly_team,
                                                      double avoid_dist)
{
    std::vector<RobotObstacle> obst;
    for (Robot r : friendly_team.getAllRobots())
    {
        obst.push_back(RobotObstacle(r, avoid_dist));
    }

    return obst;
}

std::vector<RobotObstacle> process_enemy_obstacles(const Team& enemy_team,
                                                   double avoid_dist)
{
    // No difference in implementation for now.
    return process_friendly_obstacles(enemy_team, avoid_dist);
}
