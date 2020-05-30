#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/visualizer/drawing/geom.h"
#include "software/gui/visualizer/geom/geometry_conversion.h"
#include "software/new_geom/segment.h"
#include "software/world/robot.h"

/**
 * This file contains all the functions that allow us to draw a Robot in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the robot velocity on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot's velocity
 */
void drawRobotVelocity(QGraphicsScene* scene, const Robot& robot, const QColor& color);

/**
 * Draws the robot's position on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot's position
 */
void drawRobotPosition(QGraphicsScene* scene, const Robot& robot, const QColor& color);

/**
 * Draws the robot's ID on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color of the robot being drawn
 */
void drawRobotId(QGraphicsScene* scene, const Robot& robot);

/**
 * Draws the robot on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot
 */
void drawRobot(QGraphicsScene* scene, const Robot& robot, const QColor& color);
