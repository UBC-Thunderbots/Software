#include "proto/message_translation/ssl_detection.h"

#include "shared/constants.h"

std::unique_ptr<SSLProto::SSL_DetectionBall> createSSLDetectionBall(const BallState& ball)
{
    auto detection_ball = std::make_unique<SSLProto::SSL_DetectionBall>();

    // Give all detections "perfect" confidence since we have no means of reasonably
    // esimating confidence
    detection_ball->set_confidence(1.0);
    // We don't have the information to simulate a camera and therefore the area of the
    // ball
    detection_ball->clear_area();
    auto x_position_mm = static_cast<float>(ball.position().x() * MILLIMETERS_PER_METER);
    auto y_position_mm = static_cast<float>(ball.position().y() * MILLIMETERS_PER_METER);
    detection_ball->set_x(x_position_mm);
    detection_ball->set_y(y_position_mm);
    detection_ball->set_z(
        static_cast<float>(ball.distanceFromGround() * MILLIMETERS_PER_METER));
    // We don't have the information to simulate a camera, so we use the position values
    // as pixel values
    detection_ball->set_pixel_x(x_position_mm);
    detection_ball->set_pixel_y(y_position_mm);

    return detection_ball;
}

std::unique_ptr<SSLProto::SSL_DetectionRobot> createSSLDetectionRobot(
    const RobotStateWithId& robot)
{
    auto detection_robot = std::make_unique<SSLProto::SSL_DetectionRobot>();

    // Give all detections "perfect" confidence since we have no means of reasonably
    // esimating confidence
    detection_robot->set_confidence(1.0);
    detection_robot->set_robot_id(robot.id);
    auto x_position_mm =
        static_cast<float>(robot.robot_state.position().x() * MILLIMETERS_PER_METER);
    auto y_position_mm =
        static_cast<float>(robot.robot_state.position().y() * MILLIMETERS_PER_METER);
    detection_robot->set_x(x_position_mm);
    detection_robot->set_y(y_position_mm);
    detection_robot->set_orientation(
        static_cast<float>(robot.robot_state.orientation().toRadians()));
    // We don't have the information to simulate a camera, so we use the position values
    // as pixel values
    detection_robot->set_pixel_x(x_position_mm);
    detection_robot->set_pixel_y(y_position_mm);
    detection_robot->set_height(
        static_cast<float>(ROBOT_MAX_HEIGHT_METERS * MILLIMETERS_PER_METER));

    return detection_robot;
}

std::unique_ptr<SSLProto::SSL_DetectionFrame> createSSLDetectionFrame(
    uint32_t camera_id, const Timestamp& t_capture, uint32_t frame_number,
    const std::vector<BallState>& balls,
    const std::vector<RobotStateWithId>& yellow_robots,
    const std::vector<RobotStateWithId>& blue_robots)
{
    auto detection_frame = std::make_unique<SSLProto::SSL_DetectionFrame>();

    detection_frame->set_frame_number(frame_number);
    // Assume the frame was sent instantly after it was captured
    detection_frame->set_t_capture(t_capture.toSeconds());
    detection_frame->set_t_sent(t_capture.toSeconds());
    detection_frame->set_camera_id(camera_id);

    for (const auto& ball_state : balls)
    {
        *(detection_frame->add_balls()) = *createSSLDetectionBall(ball_state);
    }

    for (const auto& yellow_robot : yellow_robots)
    {
        *(detection_frame->add_robots_yellow()) = *createSSLDetectionRobot(yellow_robot);
    }

    for (const auto& blue_robot : blue_robots)
    {
        *(detection_frame->add_robots_blue()) = *createSSLDetectionRobot(blue_robot);
    }

    return detection_frame;
}

std::vector<BallDetection> createBallDetections(
    const std::vector<SSLProto::SSL_DetectionFrame>& detections, double min_valid_x,
    double max_valid_x, bool ignore_invalid_camera_data)
{
    auto ball_detections = std::vector<BallDetection>();

    for (const auto& detection : detections)
    {
        for (const SSLProto::SSL_DetectionBall& ball : detection.balls())
        {
            // Convert all data to meters and radians
            BallDetection ball_detection{
                .position             = Point(ball.x() * METERS_PER_MILLIMETER,
                                  ball.y() * METERS_PER_MILLIMETER),
                .distance_from_ground = ball.z() * METERS_PER_MILLIMETER,
                .timestamp            = Timestamp::fromSeconds(detection.t_capture()),
                .confidence           = ball.confidence()};

            bool ignore_ball = ignore_invalid_camera_data &&
                               (min_valid_x > ball_detection.position.x() ||
                                max_valid_x < ball_detection.position.x());
            if (!ignore_ball)
            {
                ball_detections.push_back(ball_detection);
            }
        }
    }

    return ball_detections;
}

std::vector<RobotDetection> createTeamDetection(
    const std::vector<SSLProto::SSL_DetectionFrame>& detections, TeamColour team_colour,
    double min_valid_x, double max_valid_x, bool ignore_invalid_camera_data)
{
    std::vector<RobotDetection> robot_detections = std::vector<RobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto& detection : detections)
    {
        auto ssl_robots = detection.robots_blue();
        if (team_colour == TeamColour::YELLOW)
        {
            ssl_robots = detection.robots_yellow();
        }

        for (const auto& ssl_robot_detection : ssl_robots)
        {
            RobotDetection robot_detection{
                .id          = ssl_robot_detection.robot_id(),
                .position    = Point(ssl_robot_detection.x() * METERS_PER_MILLIMETER,
                                  ssl_robot_detection.y() * METERS_PER_MILLIMETER),
                .orientation = Angle::fromRadians(ssl_robot_detection.orientation()),
                .confidence  = ssl_robot_detection.confidence(),
                .timestamp   = Timestamp::fromSeconds(detection.t_capture())};

            bool ignore_robot = ignore_invalid_camera_data &&
                                (min_valid_x > robot_detection.position.x() ||
                                 max_valid_x < robot_detection.position.x());
            if (!ignore_robot)
            {
                robot_detections.push_back(robot_detection);
            }
        }
    }
    return robot_detections;
}
