#include "obstacle.h"
#include "shared/constants.h"

#define DEFAULT_VELOCITY_SCALE 2.0 // TODO: this is arbitrary

std::vector<RobotObstacle> process_friendly(Team& friendly_team, double avoid_dist)
{
    std::vector<RobotObstacle> obst;
    for (Robot r : friendly_team.getAllRobots())
    {
        obst.push_back(
            {
                .boundary = Circle(r.position(), avoid_dist + ROBOT_MAX_RADIUS),
                .velocity = Seg(r.position(), r.position() + r.velocity() * DEFAULT_VELOCITY_SCALE )
            }
        );
    }
    
    return obst;
}

std::vector<RobotObstacle> process_enemy(Team& friendly_team, double avoid_dist)
{
    // No difference in implementation for now.
    return process_friendly(friendly_team, avoid_dist);
}
