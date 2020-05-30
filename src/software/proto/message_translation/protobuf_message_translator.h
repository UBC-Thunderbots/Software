#pragma once

#include "shared/proto/tbots_software_msgs.h"
#include "software/primitive/primitive.h"

/**
 * Translates internal messages such as World, Geometry and Primitives to their
 * respective protobuf message.
 */
class ProtobufMessageTranslator
{
   public:
    /**
     * Returns a VisionMsg proto given a World.
     *
     * @param world The world msg to extract the VisionMsg from
     * @return The unique_ptr to a VisionMsg proto containing the friendly team and ball
     * information
     */
    static std::unique_ptr<VisionMsg> getVisionMsgFromWorld(World world);

    /**
     * Returns a PrimitiveMsg proto given a ConstPrimitiveVectorPtr
     *
     * @param primitives The primitives to include in the PrimitiveMsg
     * @returns The unique_ptr to a PrimitiveMsg proto containing the primitives
     */
    static std::unique_ptr<PrimitiveMsg> getPrimitiveMsgFromPrimitiveVector(
        ConstPrimitiveVectorPtr primitives);

    /**
     * Returns a PointMsg proto given a Point
     *
     * @param The Point to convert to proto
     * @return The unique_ptr to a PointMsg after conversion
     */
    static std::unique_ptr<PointMsg> getPointMsgFromPoint(Point point);

    /**
     * Returns a AngleMsg proto given an Angle
     *
     * @param The Angle to convert to proto
     * @return The unique_ptr to a AngleMsg after conversion
     */
    static std::unique_ptr<AngleMsg> getAngleMsgFromAngle(Angle angle);

    /**
     * Returns a VectorMsg proto given a Vector
     *
     * @param The Vector to convert to proto
     * @return The unique_ptr to a VectorMsg after conversion
     */
    static std::unique_ptr<VectorMsg> getVectorMsgFromVector(Vector vector);
};
