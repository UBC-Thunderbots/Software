#include "obstacle.h"
#include "constants.h"

std::vector<RobotObstacle> process_friendly(Team& friendly_team, double avoid_dist)
{
    std::vector<RobotObstacle> obst;
    for (Robot r : friendly_team.getAllRobots())
    {
        obst.push_back(
            {
                .robot = Geom::Circle(r.position(), avoid_dist + ROBOT_MAX_RADIUS),
                .velocity = Geom::Seg() // TODO: FIX
            }
        )
    }
    
    return obst;
}

std::vector<RobotObstacle> process_enemy(Team& friendly_team, double avoid_dist)
{
    // No difference in implementation for now.
    return process_friendly(friendly_team, avoid_dist);
}

std::vector<FieldBoundary> process_field(Field& field)
{
    
}