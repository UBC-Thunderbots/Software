#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/world/team.h"

/**
 * This file contains all the functions that allow us to draw a Team in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the team on the given scene.
 *
 * @param scene The scene to draw on
 * @param team The team to draw
 * @param color The color to draw the team
 */
void drawTeam(QGraphicsScene* scene, const Team& team, QColor color);
