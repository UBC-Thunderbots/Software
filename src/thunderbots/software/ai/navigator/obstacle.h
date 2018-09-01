#pragma once

#include "geom/util.h"
#include "ai/world/world.h"

constexpr double DEFAULT_AVOID_DIST = 0.15;  // meters

class RobotObstacle
{
   public:
    /**
     * Constructor.
     *
     * @param r Robot to create obstacle from.
     */
    RobotObstacle(Robot& r, double avoid_dist);

    /**
     * Returns the violation distance of the given point inside the obstacle
     * boundary; 0.0 if no violation.
     *
     * @param p point to check if violating
     *
     * @return Violation distance, defined as distance from the nearest boundary
     *         to the given point; 0.0 if no violation occured.
     */
    double getViolation(Point& p);

    /**
     * Returns the closest Point on the obstacle boundary to the given Point that
     * is not inside the boundary. If the given Point does not violate the boundary,
     * the given Point will be returned.
     *
     * @param p Point to check for
     * @return A point on the boundary nearest to p if p violates the boundary, otherwise
     *         returns p
     */
    Point getNearestValidPoint(Point& p);

    /**
     * TODO: This function may have to be rewritten depending on our needs later.
     * Checks if the given Robot will collide with the obstacle based on the current
     * velocities.
     *
     * @param r Robot to check against.
     * @return true if collision possible, false otherwise.
     */
    bool willCollide(Robot& r);

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
    Seg velocity;

    // Scaling factor for collision avoidance.
    // TODO this is arbitrary for now
    static constexpr double DEFAULT_VELOCITY_SCALE = 2.0;
};

/**
 * Creates RobotObstacles based on the friendly team robots.
 *
 * @param friendly_team The team of friendly robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 *
 * @return Vector of all friendly robots as RobotObstacles.
 */
std::vector<RobotObstacle> process_friendly_obstacles(const Team& friendly_team,
                                                      double avoid_dist);

/**
 * Creates RobotObstacles around the enemy team robots.
 *
 * @param enemy_team The team of enemy robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 *
 * @return Vector of all enemy robots as RobotObstacles.
 */
std::vector<RobotObstacle> process_enemy_obstacles(const Team& enemy_team,
                                                   double avoid_dist);
