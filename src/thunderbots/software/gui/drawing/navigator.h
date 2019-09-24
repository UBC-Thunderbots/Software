#pragma once

#include <QGraphicsScene>

#include "ai/world/ball.h"

/**
 * This file contains all the functions that allow us to draw information
 * about the AI's navigator in a QGraphicsScene in Qt
 */

/**
 * Draws the ball velocity on the given scene.
 *
 * @param scene The scene to draw on
 * @param ball The ball to draw
 * @param color The color to draw the ball's velocity
 */
void drawRobotPaths(QGraphicsScene *scene, std::vector<std::vector<Point>> paths);
