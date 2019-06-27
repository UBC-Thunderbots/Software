/**
 * An obstacle is an area to avoid based on the size of the robot and its velocity
 */
#pragma once

#include <ai/primitive/primitive.h>

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
    /**
     * Create an obstacle from the given polygon
     *
     * @param polygon
     */
    Obstacle(Polygon polygon);

    static Obstacle createRobotObstacle(const Robot& robot, bool enable_velocity_cushion);


    /*
     * Gets the boundary polygon around the given primitive that other robots
     * should not enter with a buffer scaled by width_scaling and length_scaling
     *
     * length is along the length of the primitive and the width is perpendicular to that
     *
     * @param start             start position
     * @param destination       destination position
     * @param initial_speed     initial speed of the robot
     * @param width_scaling     multiplicatively scales the width of obstacle
     * @param length_scaling    multiplicatively scales the length
     *
     * @return a rectangular Polygon to represent the boundary around the obstacle
     */
    static Obstacle createVelocityObstacleWithScalingParams(Point start, Point end,
                                                            double initial_speed,
                                                            double width_scaling,
                                                            double length_scaling);

    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer scaled by radius_cushion_scaling and
     * velocity_cushion_scaling
     *
     * The radius cushion is a hexagon such that the centre to side distance is (minimum
     * distance needed for two robots to clear each other * radius scaling)
     *
     * The velocity cushion is a rectangular projection whose length is (velocity *
     * velocity scaling) and whose width is the diameter of the radius cushion
     *
     *
     * @param robot robot to create obstacle boundary polygon around
     * @param radius_cushion_scaling multiplicatively scales the radius of the obstacle
     * @param velocity_cushion_scaling multiplicatively scales the cushion of
     * velocity of the robot
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Obstacle createRobotObstacleWithScalingParams(const Robot& robot,
                                                         double radius_cushion_scaling,
                                                         double velocity_cushion_scaling);

    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer additively expanded by
     * additional_radius_cushion_buffer and additional_velocity_cushion_buffer
     *
     * The radius cushion is a hexagon such that the centre to side distance is (minimum
     * distance needed for two robots to clear each other + radius buffer)
     *
     * The velocity cushion is a rectangular projection whose length is (velocity +
     * velocity buffer) and whose width is the diameter of the radius cushion
     *
     * @param robot robot to create obstacle boundary polygon around
     * @param additional_radius_cushion_buffer additively increases the radius of the
     * obstacle by specified amount
     * @param additional_velocity_cushion_buffer additively increases the cushion of
     * velocity of the robot
     * @param enable_velocity_cushion if true, enables the cushion of the velocity
     * of the robot as component of the obstacle
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Obstacle createRobotObstacleWithBufferParams(
        const Robot& robot, bool enable_velocity_cushion,
        double additional_radius_cushion_buffer,
        double additional_velocity_cushion_buffer);

    // TODO: @jonl112: plz add javadoc and unit test this
    static Obstacle createBallObstacle(const Ball& ball,
                                       double additional_radius_cushion_buffer,
                                       double additional_velocity_cushion_buffer);

    const Polygon& getBoundaryPolygon() const;


   private:
    static Obstacle createRobotObstacleFromPositionAndRadiusAndVelocity(
        Point position, double radius_cushion, Vector velocity_cushion_vector,
        bool enable_velocity_cushion);
    static double getRadiusCushionForHexagon(double radius);
    Polygon _polygon;
};
