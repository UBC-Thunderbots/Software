#pragma once

#include "software/geom/algorithms/closest_point.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

class EndInObstacleSampler
{
public:
    EndInObstacleSampler() = delete;

    EndInObstacleSampler(const std::vector<ObstaclePtr> obstacles, const Point& destination);

    void compute();

    void randomSample();

    void circularSample();


private:
    std::vector<ObstaclePtr> obstacles;
    Point destination;
    std::vector<Point> results;
};