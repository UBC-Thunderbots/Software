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
     * @return The VisionMsg proto containing the friendly team and ball information
     */
    static VisionMsg getVisionMsgFromWorld(World world);

    /**
     * Returns a PrimitiveMsg proto given a ConstPrimitiveVectorPtr
     *
     * @param primitives The primitives to include in the PrimitiveMsg
     * @returns The PrimitiveMsg proto containing the primitives
     */
    static PrimitiveMsg getPrimitiveMsgFromPrimitiveVector(
        ConstPrimitiveVectorPtr primitives);

    /**
     * Returns a PointMsg proto given a Point
     *
     * @param The Point to convert to proto
     * @return The PointMsg after conversion
     */
    static PointMsg getPointMsgFromPoint(Point point);

    /**
     * Returns a AngleMsg proto given an Angle
     *
     * @param The Angle to convert to proto
     * @return The AngleMsg after conversion
     */
    static AngleMsg getAngleMsgFromAngle(Angle angle);

    /**
     * Returns a VectorMsg proto given a Vector
     *
     * @param The Point to convert to proto
     * @return The PointMsg after conversion
     */
    static VectorMsg getVectorMsgFromVector(Vector vector);
};
