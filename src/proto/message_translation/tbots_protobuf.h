#pragma once

#include "proto/message_translation/tbots_geometry.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "software/world/world.h"

/**
 * Returns a TbotsProto::Vision proto given a World.
 *
 * @param world The world msg to extract the TbotsProto::Vision from
 *
 * @return The unique_ptr to a TbotsProto::Vision proto containing the friendly team and
 * ball information
 */
std::unique_ptr<TbotsProto::Vision> createVision(const World& world);

/**
 * Returns (Robot, Ball)State given a (Robot, Ball)
 *
 * @param The (Robot, Ball) to convert to State proto
 *
 * @return The unique_ptr to a (Robot, Ball)State after conversion
 */
std::unique_ptr<TbotsProto::RobotState> createRobotState(const Robot& robot);
std::unique_ptr<TbotsProto::BallState> createBallState(const Ball& ball);

/**
 * Returns a timestamp msg with the time that this function was called
 *
 * @return The unique_ptr to a TbotsProto::Timestamp with the current UTC time
 */
std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp();
