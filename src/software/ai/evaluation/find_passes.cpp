#include "software/ai/evaluation/find_passes.h"

Circle getObstacle(const Robot& robot, double radius)
{
    return Circle(robot.position(), radius);
}

std::vector<Circle> getObstacles(const std::vector<Robot>& robots, double radius)
{
    std::vector<Circle> obstacles;
    for (const Robot& robot : robots)
    {
        obstacles.push_back(getObstacle(robot, radius));
    }
    return obstacles;
}

std::vector<Robot> findOpenFriendlyRobots(const Team& friendly_team,
                                          const Team& enemy_team, double radius)
{
    std::vector<Circle> obstacles = getObstacles(enemy_team.getAllRobots(), radius);

    std::vector<Robot> open_robots;
    for (const Robot& friendly : friendly_team.getAllRobots())
    {
        if (std::all_of(obstacles.begin(), obstacles.end(),
                        [friendly](const Circle enemy_circle) {
                            return !contains(enemy_circle, friendly.position());
                        }))
        {
            open_robots.push_back(friendly);
        }
    }
    return open_robots;
}


AllPasses findAllPasses(const Robot& robot, const Team& friendly_team,
                        const Team& enemy_team, double radius)
{
    std::vector<Robot> open_robots =
        findOpenFriendlyRobots(friendly_team, enemy_team, radius);

    open_robots.erase(remove(open_robots.begin(), open_robots.end(), robot));
    std::vector<Robot> robot_obstacles = friendly_team.getAllRobots();
    robot_obstacles.insert(robot_obstacles.begin(), enemy_team.getAllRobots().begin(),
                           enemy_team.getAllRobots().end());
    robot_obstacles.erase(remove(robot_obstacles.begin(), robot_obstacles.end(), robot));

    std::vector<Circle> obstacles = getObstacles(robot_obstacles, radius);
    std::vector<Robot> direct_passes;
    std::vector<Robot> indirect_passes;
    for (const Robot& open_robot : open_robots)
    {
        Circle obstacle = getObstacle(open_robot, radius);
        obstacles.erase(remove(obstacles.begin(), obstacles.end(), obstacle));
        Segment possible_pass = Segment(robot.position(), open_robot.position());
        if (std::any_of(obstacles.begin(), obstacles.end(),
                        [possible_pass](const Circle obstacle) {
                            return intersects(possible_pass, obstacle);
                        }))
        {
            indirect_passes.push_back(open_robot);
        }
        else
        {
            direct_passes.push_back(open_robot);
        }
        obstacles.push_back(obstacle);
    }
    AllPasses all_passes{direct_passes, indirect_passes};
    return all_passes;
}
