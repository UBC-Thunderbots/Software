#pragma once

#include "software/geom/algorithms/closest_point.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

class EndInObstacleSampler
{
public:
    EndInObstacleSampler() = delete;

    EndInObstacleSampler(float rad_multiplier, float step_multiplier);

    void setRadMultiplier(float rad_multiplier);

    void setStepMultiplier(float step_multiplier);

    Point compute(const std::vector<ObstaclePtr> obstacles, const Point& destination);

//    void randomSample();

    Point circularSample(const std::vector<ObstaclePtr> obstacles, const Point& destination, float range);


private:
    float rad_multiplier;
    float step_multiplier;
    std::vector<Point> results;
};