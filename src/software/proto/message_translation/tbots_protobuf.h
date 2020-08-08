#pragma once

#include "shared/proto/tbots_software_msgs.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/primitive/primitive.h"
#include "software/proto/message_translation/tbots_geometry.h"
#include "software/world/world.h"

/**
 * Returns a TbotsProto::Vision proto given a World.
 *
 * @param world The world msg to extract the TbotsProto::Vision from
 * @return The unique_ptr to a TbotsProto::Vision proto containing the friendly team and
 * ball information
 */
std::unique_ptr<TbotsProto::Vision> createVision(const World& world);

/**
 * Returns a TbotsProto::Primitive proto given a ConstPrimitiveVectorPtr
 *
 * @param primitives The primitives to include in the TbotsProto::Primitive
 * @returns The unique_ptr to a TbotsProto::Primitive proto containing the primitives
 */
std::unique_ptr<TbotsProto::PrimitiveSet> createPrimitiveSet(
    const ConstPrimitiveVectorPtr& primitives);

/**
 * Returns (Robot, Ball)StateMsg given a (Robot, Ball)
 *
 * @param The (Robot, Ball) to convert to StateMsg proto
 * @return The unique_ptr to a (Robot, Ball)StateMsg after conversion
 */
std::unique_ptr<TbotsProto::RobotState> createRobotState(const Robot& robot);
std::unique_ptr<TbotsProto::BallState> createBallState(const Ball& ball);

/**
 * Returns a timestamp msg with the time that this function was called
 *
 * @return The unique_ptr to a TbotsProto::Timestamp with the current UTC time
 */
std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp();
