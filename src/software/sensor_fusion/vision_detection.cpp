#include "software/sensor_fusion/vision_detection.h"

VisionDetection::VisionDetection(
    const std::vector<BallDetection> &ball_detections,
    const std::vector<RobotDetection> &friendly_team_detections,
    const std::vector<RobotDetection> &enemy_team_detections,
    const std::optional<Field> &field_detection, const Timestamp latest_timestamp)
    : ball_detections(ball_detections),
      friendly_team_detections(friendly_team_detections),
      enemy_team_detections(enemy_team_detections),
      field_detection(field_detection),
      latest_timestamp(latest_timestamp)
{
}

const std::vector<BallDetection> VisionDetection::getBallDetections(void) const
{
    return ball_detections;
}

const std::vector<RobotDetection> VisionDetection::getFriendlyTeamDetections(void) const
{
    return friendly_team_detections;
}

const std::vector<RobotDetection> VisionDetection::getEnemyTeamDetections(void) const
{
    return enemy_team_detections;
}

const std::optional<Field> VisionDetection::getFieldDetection(void) const
{
    return field_detection;
}

const Timestamp VisionDetection::getLatestTimestamp(void) const
{
    return latest_timestamp;
}
