#pragma once

#include <Box2D/Box2D.h>

#include <vector>

#include "software/new_geom/point.h"
#include "software/world/robot_state.h"

/**
 * This is a helper class that assists in creating Box2D shapes for a Robot object
 */
class PhysicsRobotModel
{
   public:
    /**
     * Together these functions return 3 polygons that together make up the shape of the
     * robot. It is broken up this way because Box2D can only have convex polygons. We
     * split the body into 3 parts:
     * - The main body, which is everything behind the chicker
     * - The front-left part, which is the bit that is to the front-left of the chicker,
     * partially enclosing it
     * - The front-right part, which is the bit that is to the front-right of the chicker,
     * partially enclosing it
     *
     * These polygons are all created assuming the robot is at (0, 0) and facing along the
     * positive x-axis (ie. has an orientation of 0). This is fine because when these
     * polygons are used to create fixtures for a robot object, they are applied relative
     * to the physics body. This means that we do not need to account for any initial
     * state here.
     *
     * See the ASCII diagram below for a rough view of how the robot is created.
     * - The regions made with the '+' symbol are the front left and right bodies
     * - The region made with the 'x' symbol is the main body
     */
    /* clang-format off */
    /*
     *                            xxxxxxxxxxxxxxxxxx
     *                         x                      x
     *                      x                            x
     *                   x                                  x
     *                x                                       x
     *             x                                          x+++
     *          x                                             x+  ++
     *       x                                                x+   +  <--- Front left part
     *       x                                                x+   +       (indicated by '+')
     *       x                                                x+++++
     *       x                                                x|c|d|
     *       x                                                x|h|r|
     *       x                                                x|i|i|
     *       x                                                x|c|b|
     *       x                                                x|k|b|
     *       x                                                x|e|l|
     *       x                                                x|r|e|
     *       x                                                x+++++
     *       x                                                x+   +
     *       x                                                x+   +  <--- Front right part
     *          x                                             x+  ++       (indicated by '+')
     *             x                                          x+++
     *                x                                       x
     *                   x                                  x
     *                      x                            x
     *                         x                      x
     *                            xxxxxxxxxxxxxxxxxx
     */
    /* clang-format on */
    /**
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     *
     * @return A b2PolygonShape for the corresponding part of the robot body
     */
    static b2PolygonShape *getMainRobotBodyShape(double total_chicker_depth);
    static b2PolygonShape *getRobotBodyShapeFrontLeft(double total_chicker_depth);
    static b2PolygonShape *getRobotBodyShapeFrontRight(double total_chicker_depth);

   private:
    /**
     * A helper function that returns the points that make up the front-left shape
     * for the robot body. The points that are returned assume the robot is at (0, 0)
     * and is facing the +x axis (aka has an orientation of 0). The points are returned
     * in counter-clockwise order.
     *
     * @param robot_state The robot to create
     * @param chicker_depth How far inset into the front of the robot the chicker is
     *
     * @return The points that make up the front-left shape for the robot body
     */
    static std::vector<Point> getRobotFrontLeftShapePoints(double chicker_depth);
};
