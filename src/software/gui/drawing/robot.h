#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/drawing/colors.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/world/robot_state.h"

/**
 * This file contains all the functions that allow us to draw a Robot in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the robot velocity on the given scene.
 *
 * @param scene The scene to draw on
 * @param position The position of the robot
 * @param velocity The velocity of the robot
 * @param color The color to draw the robot's velocity
 */
void drawRobotVelocity(QGraphicsScene* scene, const Point& position,
                       const Vector& velocity, const QColor& color);

/**
 * Draws the robot's position on the given scene.
 *
 * @param scene The scene to draw on
 * @param position The position of the robot
 * @param orientation The orientation of the robot
 * @param color The color to draw the robot's position
 */
void drawRobotAtPosition(QGraphicsScene* scene, const Point& position,
                         const Angle& orientation, const QColor& color);

/**
 * Draws the robot's ID on the given scene.
 *
 * @param scene The scene to draw on
 * @param position The position of the robot
 * @param id The id of the robot
 */
void drawRobotId(QGraphicsScene* scene, const Point& position, RobotId id);

/**
 * Draws the robot on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot
 */
void drawRobot(QGraphicsScene* scene, const RobotStateWithId& robot, const QColor& color);

/**
 * Draws the robot on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot
 */
void drawRobot(QGraphicsScene* scene, const RobotDetection& robot, const QColor& color);
