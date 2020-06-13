#include "software/proto/message_translation/ssl_detection_message_translator.h"

#include <gtest/gtest.h>

#include "software/sensor_fusion/ssl_protobuf_reader.h"

TEST(SSLDetectionMessageTranslatorTest, test_create_detection_ball)
{
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    auto detection_ball = createSslDetectionBall(ball_state);
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

TEST(SSLDetectionMessageTranslatorTest, test_create_detection_robot)
{
    const RobotState state(Point(0.0, -0.5), Vector(1, 2), Angle::quarter(),
                           AngularVelocity::threeQuarter());
    const RobotStateWithId state_with_id{.id = 2, .robot_state = state};

    auto detection_robot = createSslDetectionRobot(state_with_id);
    ASSERT_TRUE(detection_robot);

    EXPECT_FLOAT_EQ(1.0, detection_robot->confidence());
    ASSERT_TRUE(detection_robot->has_robot_id());
    EXPECT_EQ(2, detection_robot->robot_id());
    EXPECT_FLOAT_EQ(0.0f, detection_robot->x());
    EXPECT_FLOAT_EQ(-500.0f, detection_robot->y());
    EXPECT_FLOAT_EQ(Angle::quarter().toRadians(), detection_robot->orientation());
    EXPECT_FLOAT_EQ(0.0f, detection_robot->pixel_x());
    EXPECT_FLOAT_EQ(-500.0f, detection_robot->pixel_y());
    ASSERT_TRUE(detection_robot->has_height());
    EXPECT_FLOAT_EQ(ROBOT_MAX_HEIGHT_METERS * MILLIMETERS_PER_METER,
                    detection_robot->height());
}

TEST(SSLDetectionMessageTranslatorTest, test_create_detection_frame)
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
        createSslDetectionFrame(camera_id, t_capture, frame_number, {ball_state},
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

TEST(SSLDetectionMessageTranslatorTest, test_convert_ball_state_to_proto_and_back)
{
    const uint32_t camera_id    = 0;
    const Timestamp t_capture   = Timestamp::fromSeconds(8.03);
    const uint32_t frame_number = 40391;
    const BallState ball_state(Point(-1.2, 0), Vector(0.01, 3), 0.2);

    auto detection_frame =
        createSslDetectionFrame(camera_id, t_capture, frame_number, {ball_state}, {}, {});
    ASSERT_TRUE(detection_frame);

    std::vector<BallDetection> ball_detections =
        SSLProtobufReader().getBallDetections({*detection_frame});
    ASSERT_EQ(1, ball_detections.size());
    BallDetection ball_detection = ball_detections.at(0);
    EXPECT_FLOAT_EQ(-1.2f, ball_detection.position.x());
    EXPECT_FLOAT_EQ(-0.0f, ball_detection.position.y());
}
