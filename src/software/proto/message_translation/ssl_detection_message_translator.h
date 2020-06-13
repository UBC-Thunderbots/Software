#pragma once

#include <memory>

#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/time/timestamp.h"
#include "software/world/ball_state.h"
#include "software/world/robot_state.h"

/**
 * Creates a DetectionBall from the given BallState.
 *
 * @param ball The ball to convert to a DetectionBall
 *
 * @return A DetectionBall representing the given BallState
 */
std::unique_ptr<SSL_DetectionBall> createSslDetectionBall(const BallState& ball);

/**
 * Creates a DetectionRobot from the given robot state.
 *
 * @param robot The robot state to convert to a DetectionRobot
 *
 * @return A DetectionRobot representing the given robot state
 */
std::unique_ptr<SSL_DetectionRobot> createSslDetectionRobot(
    const RobotStateWithId& robot);

/**
 * Creates a DetectionFrame from the given data.
 *
 * @param camera_id The id of the camera that captured this data
 * @param t_capture The time at which the data was captured
 * @param frame_number The frame number of when this data was captured
 * @param balls The ball states to add to the DetectionFrame
 * @param yellow_robots The yellow robot states to add to the DetectionFrame
 * @param blue_robots The blue robot states to add to the DetectionFrame
 *
 * @return A DetectionFrame representing the given data
 */
std::unique_ptr<SSL_DetectionFrame> createSslDetectionFrame(
    uint32_t camera_id, const Timestamp& t_capture, uint32_t frame_number,
    const std::vector<BallState>& balls,
    const std::vector<RobotStateWithId>& yellow_robots,
    const std::vector<RobotStateWithId>& blue_robots);
