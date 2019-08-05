#pragma once

#include <QGraphicsScene>
#include <QColor>
#include "ai/world/robot.h"

/**
 * This file contains all the functions that allow us to draw a Robot in a
 * QGraphicsScene in Qt
 */

// TODO: comment
void drawRobotVelocity(QGraphicsScene* scene, const Robot& robot, QColor color);
void drawRobotPosition(QGraphicsScene* scene, const Robot& robot, QPen pen, QBrush brush);
void drawRobot(QGraphicsScene* scene, const Robot& robot, QColor color);
