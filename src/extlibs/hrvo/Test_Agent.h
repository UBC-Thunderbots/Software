#pragma once

#include "software/geom/point.h"

class Test_Agent {
public:
    Point point;
    Point position() const;
    Test_Agent(double x, double y);
};
