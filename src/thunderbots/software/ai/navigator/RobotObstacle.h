#pragma once

#include "ai/world/world.h"
#include "geom/util.h"
#include "util/parameter/dynamic_parameters.h"

class RobotObstacle
{
   public:
    /**
     * Constructor.
     *
     * @param robot Robot to create obstacle from.
     * @param avoid_dist Distance to avoid obstacle by.
     */
    RobotObstacle(const Robot& robot, double avoid_dist);

    /**
     * Returns the violation distance of the given point inside the obstacle
     * boundary; 0.0 if no violation.
     *
     * @param point point to check if violating
     *
     * @return Violation distance, defined as distance from the nearest boundary
     *         to the given point; 0.0 if no violation occured.
     */
    double getViolationDistance(const Point& point);

    /**
     * Returns the closest Point on the obstacle boundary to the given Point that
     * is not inside the boundary. If the given Point does not violate the boundary,
     * the given Point will be returned.
     *
     * @param point Point to check for
     * @return A point on the boundary nearest to p if p violates the boundary, otherwise
     *         returns p
     */
    Point getNearestValidPoint(const Point& point);

    /**
     * TODO: This function may have to be rewritten depending on our needs later,
     * as part of #23:  https://github.com/UBC-Thunderbots/Software/issues/23
     *
     * Checks if the given Robot will collide with the obstacle based on the current
     * velocities.
     *
     * @param robot Robot to check against.
     * @return true if collision possible, false otherwise.
     */
    bool willCollide(const Robot& robot);

   private:
    /**
     * Represents the physical footprint of the robot, with
     * an additional avoid distance.
     */
    Circle boundary;

    /**
     * Segment extending from robot in direction of robot travel with length
     * proportional to current velocity. Used for collision avoidance.
     */
    Segment velocity;
};

/**
 * Creates RobotObstacles based on the friendly team robots.
 *
 * @param friendly_team The team of friendly robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 *
 * @return Vector of all friendly robots as RobotObstacles.
 */
std::vector<RobotObstacle> generate_friendly_obstacles(const Team& friendly_team,
                                                       double avoid_dist);

/**
 * Creates RobotObstacles around the enemy team robots.
 *
 * @param enemy_team The team of enemy robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 *
 * @return Vector of all enemy robots as RobotObstacles.
 */
std::vector<RobotObstacle> generate_enemy_obstacles(const Team& enemy_team,
                                                    double avoid_dist);
