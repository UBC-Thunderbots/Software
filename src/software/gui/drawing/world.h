#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/drawing/draw_functions.h"
#include "software/world/world.h"

/**
 * This file contains all the functions that allow us to draw a Robot in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the world on the given scene
 *
 * @param scene The scene to draw one
 * @param world The world to draw
 */
void drawWorld(QGraphicsScene* scene, const World& world);

/**
 * Returns a function that represents how to draw the provided world. Consumers
 * may call this returned function to draw the provided world onto a QGraphicsScene.
 *
 * @param world The world to create a DrawFunction for
 *
 * @return A function that represents how to draw the provided world.
 */
WorldDrawFunction getDrawWorldFunction(const World& world);
