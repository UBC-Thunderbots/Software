#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/visualizer/drawing/colors.h"
#include "software/gui/visualizer/drawing/geom.h"
#include "software/gui/visualizer/geom/geometry_conversion.h"
#include "software/new_geom/segment.h"
#include "software/world/ball.h"
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
 * @param color The color to draw the ball's velocity
 */
void drawBallVelocity(QGraphicsScene *scene, const Ball &ball, const QColor &color);

/**
 * Draws the ball's position on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 * @param color The color to draw the ball's position
 */
void drawBallPosition(QGraphicsScene *scene, const Ball &ball, const QColor &color);

/**
 * Draws the ball on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 */
void drawBall(QGraphicsScene *scene, const Ball &ball);

/**
 * Draws a cone between the ball and friendly goal posts
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw the cone for
 * @param field The field to draw the cone on
 */
void drawBallConeToFriendlyNet(QGraphicsScene *scene, const Ball &ball,
                               const Field &field);
