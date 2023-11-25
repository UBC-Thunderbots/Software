#include "proto/message_translation/ssl_detection.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(SSLDetectionTest, test_create_detection_ball)
{
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    auto detection_ball = createSSLDetectionBall(ball_state);
    ASSERT_TRUE(detection_ball);

    EXPECT_FLOAT_EQ(1.0, detection_ball->confidence());
    EXPECT_FALSE(detection_ball->has_area());
    EXPECT_FLOAT_EQ(-1200.0f, detection_ball->x());
    EXPECT_FLOAT_EQ(0.0f, detection_ball->y());
    ASSERT_TRUE(detection_ball->has_z());
    EXPECT_FLOAT_EQ(200.0f, detection_ball->z());
    EXPECT_FLOAT_EQ(-1200.0f, detection_ball->pixel_x());
    EXPECT_FLOAT_EQ(0.0f, detection_ball->pixel_y());
}

TEST(SSLDetectionTest, test_create_detection_robot)
{
    const RobotState state(Point(0.0, -0.5), Vector(1, 2), Angle::quarter(),
                           AngularVelocity::threeQuarter());
    const RobotStateWithId state_with_id{.id = 2, .robot_state = state};

    auto detection_robot = createSSLDetectionRobot(state_with_id);
    ASSERT_TRUE(detection_robot);

    EXPECT_FLOAT_EQ(1.0, detection_robot->confidence());
    ASSERT_TRUE(detection_robot->has_robot_id());
    EXPECT_EQ(2, detection_robot->robot_id());
    EXPECT_FLOAT_EQ(0.0f, detection_robot->x());
    EXPECT_FLOAT_EQ(-500.0f, detection_robot->y());
    EXPECT_FLOAT_EQ(static_cast<float>(Angle::quarter().toRadians()),
                    detection_robot->orientation());
    EXPECT_FLOAT_EQ(0.0f, detection_robot->pixel_x());
    EXPECT_FLOAT_EQ(-500.0f, detection_robot->pixel_y());
    ASSERT_TRUE(detection_robot->has_height());
    EXPECT_FLOAT_EQ(ROBOT_MAX_HEIGHT_METERS * MILLIMETERS_PER_METER,
                    detection_robot->height());
}

TEST(SSLDetectionTest, test_create_detection_frame)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(8.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    RobotState yellow_robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                                   AngularVelocity::half());
    RobotState yellow_robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                                   AngularVelocity::quarter());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
    };

    RobotState blue_robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                                 AngularVelocity::half());
    RobotState blue_robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                                 AngularVelocity::quarter());
    RobotState blue_robot_state3(Point(-1, -1), Vector(0, 2), Angle::half(),
                                 AngularVelocity::quarter());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
        RobotStateWithId{.id = 3, .robot_state = blue_robot_state3},
    };

    auto detection_frame =
        createSSLDetectionFrame(camera_id, t_capture, frame_number, {ball_state},
                                yellow_robot_states, blue_robot_states);
    ASSERT_TRUE(detection_frame);

    EXPECT_EQ(40391, detection_frame->frame_number());
    EXPECT_DOUBLE_EQ(8.03, detection_frame->t_capture());
    EXPECT_DOUBLE_EQ(8.03, detection_frame->t_sent());
    EXPECT_EQ(0, detection_frame->camera_id());
    EXPECT_EQ(1, detection_frame->balls_size());
    EXPECT_EQ(2, detection_frame->robots_yellow_size());
    EXPECT_EQ(3, detection_frame->robots_blue_size());
}

TEST(SSLDetectionTest, test_convert_ball_state_to_proto_and_back)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(8.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    auto detection_frame =
        createSSLDetectionFrame(camera_id, t_capture, frame_number, {ball_state}, {}, {});
    ASSERT_TRUE(detection_frame);

    std::vector<BallDetection> ball_detections = createBallDetections({*detection_frame});
    ASSERT_EQ(1, ball_detections.size());
    BallDetection ball_detection = ball_detections.at(0);
    EXPECT_FLOAT_EQ(-1.2f, static_cast<float>(ball_detection.position.x()));
    EXPECT_FLOAT_EQ(-0.0f, static_cast<float>(ball_detection.position.y()));
    EXPECT_FLOAT_EQ(0.2f, static_cast<float>(ball_detection.distance_from_ground));
}

TEST(SSLDetectionTest, test_convert_invalid_position_ball_state_to_proto_and_back)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(8.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.3);

    auto detection_frame =
        createSSLDetectionFrame(camera_id, t_capture, frame_number, {ball_state}, {}, {});
    ASSERT_TRUE(detection_frame);

    std::vector<BallDetection> ball_detections =
        createBallDetections({*detection_frame}, 0.0, 1.1, true);
    ASSERT_EQ(0, ball_detections.size());
}

TEST(SSLDetectionTest, test_convert_valid_position_ball_state_to_proto_and_back)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(3.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(0.2, 2), Vector(0.01, 3), 0.2);

    auto detection_frame =
        createSSLDetectionFrame(camera_id, t_capture, frame_number, {ball_state}, {}, {});
    ASSERT_TRUE(detection_frame);

    std::vector<BallDetection> ball_detections =
        createBallDetections({*detection_frame}, 0.0, 1.1, true);
    ASSERT_EQ(1, ball_detections.size());
    BallDetection ball_detection = ball_detections.at(0);
    EXPECT_FLOAT_EQ(0.2f, static_cast<float>(ball_detection.position.x()));
    EXPECT_FLOAT_EQ(2.0f, static_cast<float>(ball_detection.position.y()));
    EXPECT_FLOAT_EQ(0.2f, static_cast<float>(ball_detection.distance_from_ground));
}

TEST(SSLDetectionTest, test_convert_robot_states_to_proto_and_back)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(8.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    RobotState yellow_robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                                   AngularVelocity::half());
    RobotState yellow_robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                                   AngularVelocity::quarter());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
    };

    RobotState blue_robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                                 AngularVelocity::half());
    RobotState blue_robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                                 AngularVelocity::quarter());
    RobotState blue_robot_state3(Point(-1, -1), Vector(0, 2), Angle::half(),
                                 AngularVelocity::quarter());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
        RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
        RobotStateWithId{.id = 3, .robot_state = blue_robot_state3},
    };

    auto detection_frame =
        createSSLDetectionFrame(camera_id, t_capture, frame_number, {ball_state},
                                yellow_robot_states, blue_robot_states);
    ASSERT_TRUE(detection_frame);
    std::vector<SSLProto::SSL_DetectionFrame> detection_frames({*detection_frame});
    auto yellow_team_detections =
        createTeamDetection(detection_frames, TeamColour::YELLOW);
    ASSERT_EQ(2, yellow_team_detections.size());
    ASSERT_EQ(1.0, yellow_team_detections[0].confidence);
    ASSERT_EQ(1.0, yellow_team_detections[1].confidence);
    ASSERT_EQ(Point(1, 0), yellow_team_detections[0].position);
    ASSERT_EQ(Point(0, 0), yellow_team_detections[1].position);
    ASSERT_TRUE(TestUtil::equalWithinTolerance(Angle::quarter(),
                                               yellow_team_detections[0].orientation,
                                               Angle::fromDegrees(0.5)));
    ASSERT_TRUE(TestUtil::equalWithinTolerance(
        Angle::half(), yellow_team_detections[1].orientation, Angle::fromDegrees(0.5)));
    ASSERT_EQ(t_capture, yellow_team_detections[0].timestamp);
    ASSERT_EQ(t_capture, yellow_team_detections[1].timestamp);
    ASSERT_EQ(1, yellow_team_detections[0].id);
    ASSERT_EQ(2, yellow_team_detections[1].id);

    auto blue_team_detections = createTeamDetection(detection_frames, TeamColour::BLUE);
    ASSERT_EQ(3, blue_team_detections.size());
    ASSERT_EQ(1.0, blue_team_detections[0].confidence);
    ASSERT_EQ(1.0, blue_team_detections[1].confidence);
    ASSERT_EQ(1.0, blue_team_detections[2].confidence);
    ASSERT_EQ(Point(1, 0), blue_team_detections[0].position);
    ASSERT_EQ(Point(0, 0), blue_team_detections[1].position);
    ASSERT_EQ(Point(-1, -1), blue_team_detections[2].position);
    ASSERT_TRUE(TestUtil::equalWithinTolerance(
        Angle::quarter(), blue_team_detections[0].orientation, Angle::fromDegrees(0.5)));
    ASSERT_TRUE(TestUtil::equalWithinTolerance(
        Angle::half(), blue_team_detections[1].orientation, Angle::fromDegrees(0.5)));
    ASSERT_TRUE(TestUtil::equalWithinTolerance(
        Angle::half(), blue_team_detections[2].orientation, Angle::fromDegrees(0.5)));
    ASSERT_EQ(t_capture, blue_team_detections[0].timestamp);
    ASSERT_EQ(t_capture, blue_team_detections[1].timestamp);
    ASSERT_EQ(t_capture, blue_team_detections[2].timestamp);
    ASSERT_EQ(1, blue_team_detections[0].id);
    ASSERT_EQ(2, blue_team_detections[1].id);
    ASSERT_EQ(3, blue_team_detections[2].id);
}
