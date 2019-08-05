#pragma once

#include <QGraphicsScene>
#include "ai/world/ball.h"

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
void drawBall(QGraphicsScene* scene, const Ball& ball);
