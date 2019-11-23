#include "software/sensor_fusion/detection.h"

VisionDetection::VisionDetection(std::vector<BallDetection> ball_detections,
                                 std::vector<RobotDetection> friendly_team_detections,
                                 std::vector<RobotDetection> enemy_team_detections)
{
}

std::vector<BallDetection> VisionDetection::getBallDetections(void)
{
    return ball_detections;
}

std::vector<RobotDetection> VisionDetection::getFriendlyTeamDetections(void)
{
    return friendly_team_detections;
}

std::vector<RobotDetection> VisionDetection::getEnemyTeamDetections(void)
{
    return enemy_team_detections;
}
