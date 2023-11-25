#pragma once

#include "shared/constants.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/circle.h"
#include "software/geom/segment.h"
#include "software/world/world.h"

struct AllPasses
{
    // A vector containing the receiving robots of all indirect passes to open robots on
    // the friendly_team, a indirect pass is a pass where there is an obstacle between the
    // passing and receiving robots
    std::vector<Robot> direct_passes;

    // A vector containing receiving robots of all direct passes to open robots on the
    // friendly_team, a direct pass is a pass where there is no obstacle between the
    // passing and receiving robots
    std::vector<Robot> indirect_passes;
};



/**
 * Finds all direct and indirect passes to open robots.
 *
 * @param robot The robot that could take the passes
 * @param friendly_team The same team of the robot with the possession of the ball
 * @param enemy_team The opposing team of the robot with the possession of the ball
 * @param radius The radius around a robot to be treated as an obstacle
 *
 * @return A vector containing receiving robots of all direct and indirect passes to open
 * robots on the friendly_team
 */
AllPasses findAllPasses(const Robot& robot, const Team& friendly_team,
                        const Team& enemy_team,
                        double radius = ROBOT_MAX_RADIUS_METERS * 1.5);


/**
 * Return a circle that is treated as an obstacle centered at the robot position with
 * the given radius
 *
 * @param robot The robot considered as an obstacle
 * @param radius The radius around the robot
 *
 * @return A circle centered at the robot with the given radius
 */
Circle getObstacle(const Robot& robot, double radius);


/**
 * Return a vector of circles that are treated as an obstacle centered at the robot
 * position with the given radius
 *
 * @param robots The robots considered as obstacles
 * @param radius The radius around the robots
 *
 * @return A vector of circles centered at the robots with the given radius
 */
std::vector<Circle> getObstacles(const std::vector<Robot>& robots, double radius);

/**
 * Return a vector of robots on the friendly team such that each robot is at least
 * a radius away from an enemy robot.
 *
 * @param Team The friendly team
 * @param Team The enemy team
 * @param radius The distance threshold between friendly and enemy robots
 *
 * @return A vector of friendly robots that are considered open
 */
std::vector<Robot> findOpenFriendlyRobots(const Team& friendly_team,
                                          const Team& enemy_team, double radius);
