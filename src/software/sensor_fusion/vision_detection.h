#pragma once

#include <vector>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/sensor_fusion/ball_detection.h"
#include "software/sensor_fusion/robot_detection.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * VisionDetection represents ball and robot detections
 * and is used to pass data from backend to sensor fusion
 */
class VisionDetection
{
   public:
    /**
     * Creates a new VisionDetection object
     *
     * @param ball_detections list of ball detections
     * @param friendly_team_detections list of robot detections for friendly team
     * @param enemy_team_detections list of robot detections for enemy team
     * @param latest_timestamp latest timestamp
     *
     */
    VisionDetection(const std::vector<BallDetection> &ball_detections,
                    const std::vector<RobotDetection> &friendly_team_detections,
                    const std::vector<RobotDetection> &enemy_team_detections,
                    const Timestamp latest_timestamp);

    /**
     * Gets ball detections
     *
     * @return ball detections
     */
    const std::vector<BallDetection> getBallDetections(void) const;

    /**
     * Gets friendly team detections
     *
     * @return friendly team detections
     */
    const std::vector<RobotDetection> getFriendlyTeamDetections(void) const;

    /**
     * Gets enemy team detections
     *
     * @return enemy team detections
     */
    const std::vector<RobotDetection> getEnemyTeamDetections(void) const;

    /**
     * Gets latest timestamp
     *
     * @return latest timestamp
     */
    const Timestamp getLatestTimestamp(void) const;

   private:
    std::vector<BallDetection> ball_detections;
    std::vector<RobotDetection> friendly_team_detections;
    std::vector<RobotDetection> enemy_team_detections;
    Timestamp latest_timestamp;
};
