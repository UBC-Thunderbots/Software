#pragma once

#include "software/new_geom/point.h"
#include "software/util/time/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_DetectionBall directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
struct BallDetection
{
    // The position of the ball detection on the field, in meters
    Point position;
    // The timestamp of the detection. This is the timestamp for when the camera frame
    // containing the detection was captured
    Timestamp timestamp;

    bool operator<(const BallDetection &b) const
    {
        return timestamp < b.timestamp;
    }
};
