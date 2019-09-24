#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/world/world.h"

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
