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
 * @param slow_colour The velocity line colour when the speed is slow
 * @param fast_colour The velocity line colour when the speed is fast
 * @param robot_constants The robot constants
 */
void drawRobotVelocity(QGraphicsScene* scene, const Point& position,
                       const Vector& velocity, const QColor& slow_colour,
                       const QColor& fast_colour,
                       const RobotConstants_t& robot_constants);

/**
 * Draws the robot's position on the given scene.
 *
 * @param scene The scene to draw on
 * @param position The position of the robot
 * @param orientation The orientation of the robot
 * @param color The color to draw the robot's position
 * @param robot_constants The robot constants
 */
void drawRobotAtPosition(QGraphicsScene* scene, const Point& position,
                         const Angle& orientation, const QColor& color,
                         const RobotConstants_t& robot_constants);

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
 * @param robot_constants The robot constants
 */
void drawRobot(QGraphicsScene* scene, const RobotStateWithId& robot, const QColor& color,
               const RobotConstants_t& robot_constants);

/**
 * Draws the robot on the given scene.
 *
 * @param scene The scene to draw on
 * @param robot The robot to draw
 * @param color The color to draw the robot
 * @param robot_constants The robot constants
 */
void drawRobot(QGraphicsScene* scene, const RobotDetection& robot, const QColor& color,
               const RobotConstants_t& robot_constants);
