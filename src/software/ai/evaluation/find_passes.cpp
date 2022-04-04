#include "software/ai/evaluation/find_passes.h"

Circle getObstacle(const Robot& robot, double radius)
{
    return Circle(robot.position(), radius);
}

std::vector<Circle> getObstacles(const std::vector<Robot>& robots, double radius)
{
    std::vector<Circle> obstacles;
    for (Robot robot : robots)
    {
        obstacles.push_back(getObstacle(robot, radius));
    }
    return obstacles;
}

// std::vector<Circle> getObstacles(const Team& team, double radius, std::vector<Robot>
// ignore){
//     std::vector<Circle> obstacles;
//     for(Robot robot: team.getAllRobots()){
//         if(std::find(std::begin(ignore),std::end(ignore), robot) == std::end(ignore)){
//             obstacles.push_back(Circle(robot.position(),radius));
//         }
//     }
//     return obstacles;
// }

std::vector<Robot> findOpenRobots(const Team& friendly_team, const Team& enemy_team,
                                  double radius)
{
    std::vector<Circle> obstacles = getObstacles(enemy_team.getAllRobots(), radius);

    std::vector<Robot> open_robots;
    for (Robot friendly : friendly_team.getAllRobots())
    {
        if (std::all_of(obstacles.begin(), obstacles.end(),
                        [friendly](Circle enemy_circle) {
                            return !contains(enemy_circle, friendly.position());
                        }))
        {
            open_robots.push_back(friendly);
        }
    }
    return open_robots;
}

std::vector<Pass> findDirectPasses(const Robot& robot, const Team& friendly_team,
                                   const Team& enemy_team)
{
    std::vector<Robot> open_robots =
        findOpenRobots(friendly_team, enemy_team, ROBOT_MAX_RADIUS_METERS * 1.5);
    remove(open_robots.begin(), open_robots.end(), robot);

    std::vector<Robot> robot_obstacles = friendly_team.getAllRobots();
    robot_obstacles.insert(robot_obstacles.begin(), enemy_team.getAllRobots().begin(),
                           enemy_team.getAllRobots().end());
    remove(robot_obstacles.begin(), robot_obstacles.end(), robot);

    std::vector<Circle> obstacles =
        getObstacles(robot_obstacles, ROBOT_MAX_RADIUS_METERS * 1.5);
    std::vector<Pass> direct_passes;
    for (Robot open_robot : open_robots)
    {
        Circle obstacle = getObstacle(open_robot, ROBOT_MAX_RADIUS_METERS * 1.5);
        remove(obstacles.begin(), obstacles.end(), obstacle);
        Segment possible_pass = Segment(robot.position(), open_robot.position());
        if (std::all_of(obstacles.begin(), obstacles.end(),
                        [possible_pass](Circle obstacle) {
                            return !intersects(possible_pass, obstacle);
                        }))
        {
            direct_passes.push_back(Pass(robot.position(), open_robot.position(),
                                         BALL_MAX_SPEED_METERS_PER_SECOND));
        }
        obstacles.push_back(obstacle);
    }
    return direct_passes;
}

std::vector<Pass> findIndirectPasses(const Robot& robot, const Team& friendly_team,
                                     const Team& enemy_team)
{
    std::vector<Robot> open_robots =
        findOpenRobots(friendly_team, enemy_team, ROBOT_MAX_RADIUS_METERS * 1.5);
    remove(open_robots.begin(), open_robots.end(), robot);

    std::vector<Robot> robot_obstacles = friendly_team.getAllRobots();
    robot_obstacles.insert(robot_obstacles.begin(), enemy_team.getAllRobots().begin(),
                           enemy_team.getAllRobots().end());
    remove(robot_obstacles.begin(), robot_obstacles.end(), robot);

    std::vector<Circle> obstacles =
        getObstacles(robot_obstacles, ROBOT_MAX_RADIUS_METERS * 1.5);
    std::vector<Pass> indirect_passes;
    for (Robot open_robot : open_robots)
    {
        Circle obstacle = getObstacle(open_robot, ROBOT_MAX_RADIUS_METERS * 1.5);
        remove(obstacles.begin(), obstacles.end(), obstacle);
        Segment possible_pass = Segment(robot.position(), open_robot.position());
        if (std::any_of(obstacles.begin(), obstacles.end(),
                        [possible_pass](Circle obstacle) {
                            return intersects(possible_pass, obstacle);
                        }))
        {
            indirect_passes.push_back(Pass(robot.position(), open_robot.position(),
                                           BALL_MAX_SPEED_METERS_PER_SECOND));
        }
        obstacles.push_back(obstacle);
    }
    return indirect_passes;
}
