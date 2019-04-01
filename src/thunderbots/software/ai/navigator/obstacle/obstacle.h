/**
 * An obstacle is an area to avoid based on the size of the robot and its velocity
 */
#pragma once

#include "ai/world/ball.h"
#include "ai/world/robot.h"
#include "geom/point.h"
#include "geom/polygon.h"
#include "geom/util.h"

class Obstacle
{
public:
    static Obstacle createPlaceholderObstacle();
    static Obstacle createRobotObstacle(const Robot& robot);
    static Obstacle createBallObstacle(const Ball& ball);
    const Polygon &getBoundaryPolygon() const;
private:
    // placeholder obstacle is a 1x1 square
    Obstacle();
    Polygon _polygon;
};
