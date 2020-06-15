#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/time/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_DetectionRobot data directly
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

    bool operator<(const RobotDetection &r) const
    {
        return timestamp < r.timestamp;
    }
};

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
    double confidence;

    bool operator<(const BallDetection &b) const
    {
        return timestamp < b.timestamp;
    }
};
