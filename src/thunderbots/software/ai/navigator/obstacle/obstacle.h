/**
 * An obstacle is an area to avoid based on the size of the robot and its velocity
 */
#pragma once

#include "ai/world/ball.h"
#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/polygon.h"
#include "geom/util.h"
#include "shared/constants.h"

class Obstacle
{
   public:
    static Obstacle createPlaceholderObstacle();

    static Obstacle createRobotObstacle(const Robot& robot, bool enable_velocity_cushion);

    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer scaled by radius_scaling and
     * velocity_cushion_scaling
     *
     * @param robot robot to create obstacle boundary polygon around
     * @param radius_scaling multiplicatively scales the radius of the obstacle
     * @param velocity_cushion_scaling multiplicatively scales the cushion of
     * velocity of the robot
     * @param enable_velocity_cushion if true, enables the cushion of the velocity
     * of the robot as component of the obstacle
     *
     * @note the radius is by default the minimum distance needed for two robots to clear
     * each other
     * @note the velocity cushion is by default the distance that can be travelled in
     * one tick
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Obstacle createRobotObstacleWithScalingParams(const Robot& robot,
                                                         bool enable_velocity_cushion,
                                                         double radius_cushion_scaling,
                                                         double velocity_cushion_scaling,
                                                         double tick_length = 1.0);

    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer scaled by radius_scaling and
     * velocity_cushion_scaling
     *
     * @param robot robot to create obstacle boundary polygon around
     * @param radius_buffer additively increases the radius of the obstacle by specified
     * amount
     * @param additional_velocity_cushion_buffer additively increases the cushion of
     * velocity of the robot
     * @param enable_velocity_cushion if true, enables the cushion of the velocity
     * of the robot as component of the obstacle
     *
     * @note the radius is by default the minimum distance needed for two robots to clear
     * each other
     * @note the velocity cushion is by default the distance that can be travelled in
     * one tick
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Obstacle createRobotObstacleWithBufferParams(
        const Robot& robot, bool enable_velocity_cushion,
        double additional_radius_cushion_buffer,
        double additional_velocity_cushion_buffer, double tick_length = 1.0);

    static Obstacle createBallObstacle(const Ball& ball);

    const Polygon& getBoundaryPolygon() const;

   private:
    // placeholder obstacle is a 1x1 square
    Obstacle();
    Obstacle(Polygon polygon);
    static Obstacle createRobotObstacleFromPositionAndRadiusAndVelocity(
        Point position, double radius_cushion, Vector velocity_cushion_vector,
        bool enable_velocity_cushion);
    static double getRadiusForHexagon(double radius);
    Polygon _polygon;
};
