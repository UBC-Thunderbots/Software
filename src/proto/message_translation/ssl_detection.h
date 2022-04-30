#pragma once

#include <limits>
#include <memory>

#include "proto/ssl_vision_detection.pb.h"
#include "software/sensor_fusion/filter/vision_detection.h"
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
std::unique_ptr<SSLProto::SSL_DetectionBall> createSSLDetectionBall(
    const BallState& ball);

/**
 * Creates a DetectionRobot from the given robot state.
 *
 * @param robot The robot state to convert to a DetectionRobot
 *
 * @return A DetectionRobot representing the given robot state
 */
std::unique_ptr<SSLProto::SSL_DetectionRobot> createSSLDetectionRobot(
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
std::unique_ptr<SSLProto::SSL_DetectionFrame> createSSLDetectionFrame(
    uint32_t camera_id, const Timestamp& t_capture, uint32_t frame_number,
    const std::vector<BallState>& balls,
    const std::vector<RobotStateWithId>& yellow_robots,
    const std::vector<RobotStateWithId>& blue_robots);

/**
 * Reads the ball data contained in the list of SSL detection frames and returns the
 * ball detections
 *
 * Filters out ball detections that are less than min_valid_x and greater than max_valid_x
 * if ignore_invalid_camera_data is true
 *
 * @param detections A list of new DetectionFrames containing ball data
 * @param min_valid_x min valid x value
 * @param max_valid_x max valid x value
 * @param ignore_invalid_camera_data whether or not to ignore ball outside of valid x
 * range
 *
 * @return all the valid ball detections contained in the ssl detection frames
 */
std::vector<BallDetection> createBallDetections(
    const std::vector<SSLProto::SSL_DetectionFrame>& detections,
    double min_valid_x              = std::numeric_limits<double>::min(),
    double max_valid_x              = std::numeric_limits<double>::max(),
    bool ignore_invalid_camera_data = false);

/**
 * Reads the robot data for the given team contained in the list of DetectionFrames
 * and returns the most up to date detections of the given colour team
 *
 * Filters out robot detections that are less than min_valid_x and greater than
 * max_valid_x if ignore_invalid_camera_data is true
 *
 * @param detections A list of new DetectionFrames containing given team robot data
 * @param team_colour the team colour to get detections for
 * @param min_valid_x min valid x value
 * @param max_valid_x max valid x value
 * @param ignore_invalid_camera_data whether or not to ignore robot outside of valid x
 * range
 *
 *
 * @return The most up to date detections of the given team given the new
 * DetectionFrame information
 */
std::vector<RobotDetection> createTeamDetection(
    const std::vector<SSLProto::SSL_DetectionFrame>& detections, TeamColour team_colour,
    double min_valid_x              = std::numeric_limits<double>::min(),
    double max_valid_x              = std::numeric_limits<double>::max(),
    bool ignore_invalid_camera_data = false);
