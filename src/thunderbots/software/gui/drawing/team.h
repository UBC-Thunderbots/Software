#pragma once

#include <QGraphicsScene>
#include "ai/world/team.h"

/**
 * This file contains all the functions that allow us to draw a Team in a
 * QGraphicsScene in Qt
 */

// TODO: comment
void drawTeam(QGraphicsScene* scene, const Team& team, QColor color);
void drawFriendlyTeam(QGraphicsScene* scene, const Team& team);
void drawEnemyTeam(QGraphicsScene* scene, const Team& team);
