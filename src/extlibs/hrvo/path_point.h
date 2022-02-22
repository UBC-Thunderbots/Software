#pragma once
#include <vector>

#include "extlibs/hrvo/vector2.h"

class Simulator;

class PathPoint
{
    public:
        explicit PathPoint(const Vector2 &position);
        explicit PathPoint(const Vector2 &position, const float speed_at_position);

    private:
        Vector2 position_;
        float speed_at_destination;

    friend class Path;

};