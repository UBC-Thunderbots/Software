#pragma once

#include "shared/proto/geometry.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/primitive/primitive.h"
#include "software/world/world.h"

namespace ProtobufMessageTranslation
{
    /**
     * Returns a VisionMsg proto given a World.
     *
     * @param world The world msg to extract the VisionMsg from
     * @return The unique_ptr to a VisionMsg proto containing the friendly team and ball
     * information
     */
    std::unique_ptr<VisionMsg> getVisionMsgFromWorld(const World& world);

    /**
     * Returns a PrimitiveMsg proto given a ConstPrimitiveVectorPtr
     *
     * @param primitives The primitives to include in the PrimitiveMsg
     * @returns The unique_ptr to a PrimitiveMsg proto containing the primitives
     */
    std::unique_ptr<PrimitiveMsg> getPrimitiveMsgFromPrimitiveVector(
        const ConstPrimitiveVectorPtr& primitives);

    /**
     * Returns (Robot, Ball)StateMsg given a (Robot, Ball)
     *
     * @param The (Robot, Ball) to convert to StateMsg proto
     * @return The unique_ptr to a (Robot, Ball)StateMsg after conversion
     */
    std::unique_ptr<RobotStateMsg> getRobotStateMsgFromRobot(const Robot& robot);
    std::unique_ptr<BallStateMsg> getBallStateMsgFromBall(const Ball& ball);

    /**
     * Internal geometry types to protobuf msg conversions
     *
     * @param The geom type (Point, Angle, Vector) to convert to proto
     * @return The unique_ptr to the converted GeomMsg
     */
    std::unique_ptr<PointMsg> getPointMsgFromPoint(const Point& point);
    std::unique_ptr<AngleMsg> getAngleMsgFromAngle(const Angle& angle);
    std::unique_ptr<VectorMsg> getVectorMsgFromVector(const Vector& vector);

    /**
     * Returns a timestamp msg with the time that this function was called
     *
     * @return The unique_ptr to a TimestampMsg with the current UTC time
     */
    std::unique_ptr<TimestampMsg> getCurrentTimestampMsg();

}  // namespace ProtobufMessageTranslation
