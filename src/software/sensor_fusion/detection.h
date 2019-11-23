#pragma once

#include <vector>

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

class VisionDetection
{
   public:
    /**
     * Creates a new VisionDetection object
     *
     * @param ball_detections list of ball detections
     * @param friendly_team_detections list of robot detections for friendly team
     * @param enemy_team_detections list of robot detections for enemy team
     *
     */
    VisionDetection(std::vector<BallDetection> ball_detections,
                    std::vector<RobotDetection> friendly_team_detections,
                    std::vector<RobotDetection> enemy_team_detections);

    /**
     * Gets ball detections
     *
     * @return ball detections
     */
    std::vector<BallDetection> getBallDetections(void);

    /**
     * Gets friendly team detections
     *
     * @return friendly team detections
     */
    std::vector<RobotDetection> getFriendlyTeamDetections(void);

    /**
     * Gets enemy team detections
     *
     * @return enemy team detections
     */
    std::vector<RobotDetection> getEnemyTeamDetections(void);

   private:
    std::vector<BallDetection> ball_detections;
    std::vector<RobotDetection> friendly_team_detections;
    std::vector<RobotDetection> enemy_team_detections;
};
