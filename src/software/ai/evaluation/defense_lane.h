#pragma once

#include "software/geom/point.h"
#include "software/geom/segment.h"

struct DefenseLane
{
    Segment lane;

    unsigned int expected_threat;
};

struct DefensePosition
{
    Point position;

    unsigned int expected_threat;

    bool is_crease_defense;
};
