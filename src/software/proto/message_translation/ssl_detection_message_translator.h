#pragma once

#include <memory>

#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/world/ball_state.h"
#include "software/world/robot_state.h"
#include "software/time/timestamp.h"

std::unique_ptr<SSL_DetectionBall> createDetectionBall(const BallState& ball);
std::unique_ptr<SSL_DetectionRobot> createDetectionRobot(const RobotStateWithId& robot);
std::unique_ptr<SSL_DetectionFrame> createDetectionFrame(uint32_t camera_id, const Timestamp& t_capture, uint32_t frame_number, const std::vector<BallState>& balls, const std::vector<RobotStateWithId>& yellow_robots, const std::vector<RobotStateWithId>& blue_robots);

