#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/drawing/colors.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/world/ball_state.h"
#include "software/world/field.h"

/**
 * This file contains all the functions that allow us to draw a Ball in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the ball velocity on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 * @param slow_colour The velocity line colour when the velocity is slow
 * @param fast_colour The velocity line colour when the velocity is fast
 */
void drawBallVelocity(QGraphicsScene *scene, const Point &position,
                      const Vector &velocity, const QColor &slow_colour,
                      const QColor &fast_colour);

/**
 * Draws the ball's position on the given scene.
 *
 * @param scene The scene to draw on
 * @param position The position of the ball
 * @param distance_from_ground the distance of the ball off the ground
 * @param color The color to draw the ball's position
 */
void drawBallPosition(QGraphicsScene *scene, const Point &position,
                      double distance_from_ground, QColor color);

/**
 * Draws the ball on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 */
void drawBall(QGraphicsScene *scene, const BallState &ball);

/**
 * Draws the ball on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 */
void drawBall(QGraphicsScene *scene, const BallDetection &ball);

/**
 * Draws a cone between the ball and friendly goal posts
 *
 * @param scene The scene to draw on
 * @param position The position of the ball
 * @param field The field to draw the cone on
 */
void drawBallConeToFriendlyNet(QGraphicsScene *scene, const Point &position,
                               const Field &field);
