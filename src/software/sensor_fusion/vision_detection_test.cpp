#include "software/sensor_fusion/vision_detection.h"

#include <gtest/gtest.h>

TEST(VisionDetectionTest, test_getters_of_vision_detection)
{
    std::vector<BallDetection> ball_detections;
    std::vector<RobotDetection> friendly_team_detections;
    std::vector<RobotDetection> enemy_team_detections;
    Timestamp latest_timestamp = Timestamp::fromSeconds(1.0);

    VisionDetection vision_detection(ball_detections, friendly_team_detections,
                                     enemy_team_detections, latest_timestamp);

    EXPECT_EQ(0, vision_detection.getBallDetections().size());
    EXPECT_EQ(0, vision_detection.getFriendlyTeamDetections().size());
    EXPECT_EQ(0, vision_detection.getEnemyTeamDetections().size());
    EXPECT_EQ(latest_timestamp, vision_detection.getLatestTimestamp());
}
