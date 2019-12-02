#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/util/time/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_Detection data directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
struct RobotDetection
{
    unsigned int id;
    Point position;
    Angle orientation;
    double confidence;
    Timestamp timestamp;
};
