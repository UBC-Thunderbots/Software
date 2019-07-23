#pragma once

#include <QGraphicsScene>
#include "software/ai/world/world.h"

void drawWorld(QGraphicsScene* scene, const World& world);

void drawField(QGraphicsScene* scene, const Field& field);

void drawTeam(QGraphicsScene* scene, const Team& team, const QColor& color);
void drawFriendlyTeam(QGraphicsScene* scene, const Team& team);
void drawEnemyTeam(QGraphicsScene* scene, const Team& team);
void drawRobot(QGraphicsScene* scene, const Robot& robot, const QColor& color);
