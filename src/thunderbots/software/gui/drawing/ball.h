#pragma once

#include <QGraphicsScene>
#include "ai/world/ball.h"

// TODO: comment
void drawBallVelocity(QGraphicsScene* scene, const Ball& ball, QPen pen);
void drawBallPosition(QGraphicsScene* scene, const Ball& ball, QPen pen, QBrush brush);
void drawBall(QGraphicsScene* scene, const Ball& ball);
