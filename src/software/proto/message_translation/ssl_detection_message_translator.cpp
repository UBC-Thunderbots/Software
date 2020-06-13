#include "software/proto/message_translation/ssl_detection_message_translator.h"

#include "shared/constants.h"

std::unique_ptr<SSL_DetectionBall> createSslDetectionBall(const BallState& ball)
{
    auto detection_ball = std::make_unique<SSL_DetectionBall>();

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
    detection_ball->set_z(static_cast<float>(ball.height() * MILLIMETERS_PER_METER));
    // We don't have the information to simulate a camera, so we use the position values
    // as pixel values
    detection_ball->set_pixel_x(x_position_mm);
    detection_ball->set_pixel_y(y_position_mm);

    return std::move(detection_ball);
}

std::unique_ptr<SSL_DetectionRobot> createSslDetectionRobot(const RobotStateWithId& robot)
{
    auto detection_robot = std::make_unique<SSL_DetectionRobot>();

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

    return std::move(detection_robot);
}

std::unique_ptr<SSL_DetectionFrame> createSslDetectionFrame(
    uint32_t camera_id, const Timestamp& t_capture, uint32_t frame_number,
    const std::vector<BallState>& balls,
    const std::vector<RobotStateWithId>& yellow_robots,
    const std::vector<RobotStateWithId>& blue_robots)
{
    auto detection_frame = std::make_unique<SSL_DetectionFrame>();

    detection_frame->set_frame_number(frame_number);
    // Assume the frame was sent instantly after it was captured
    detection_frame->set_t_capture(t_capture.getSeconds());
    detection_frame->set_t_sent(t_capture.getSeconds());
    detection_frame->set_camera_id(camera_id);

    for (const auto& ball_state : balls)
    {
        auto ball = detection_frame->add_balls();
        *ball     = *(createSslDetectionBall(ball_state).release());
    }

    for (const auto& yellow_robot : yellow_robots)
    {
        auto robot = detection_frame->add_robots_yellow();
        *robot     = *(createSslDetectionRobot(yellow_robot).release());
    }

    for (const auto& blue_robot : blue_robots)
    {
        auto robot = detection_frame->add_robots_blue();
        *robot     = *(createSslDetectionRobot(blue_robot).release());
    }

    return std::move(detection_frame);
}
