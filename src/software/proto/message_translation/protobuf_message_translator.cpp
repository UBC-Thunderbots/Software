#include "software/proto/protobuf_primitive_visitor.h"

VisionMsg ProtobufMessageTranslator::getVisionMsgFromWorld(World world) {

    world.


}

PrimitiveMsg ProtoMessageTranslator::getPrimitiveMsgFromPrimitiveVector(
        ConstPrimitiveVectorPtr primitives)
{
    PrimitiveMsg
}

PointMsg ProtoMessageTranslator::getPointMsgFromPoint(Point point) {
    PointMsg point_msg;
    point_msg.set_x(point.x());
    point_msg.set_y(point.y());
    return point_msg;
}

AngleMsg ProtoMessageTranslator::getAngleMsgFromAngle(Angle angle) {

    AngleMsg angle_msg;
    angle_msg.set_radians(angle.toRadians());
    return angle_msg;

}

VectorMsg ProtoMessageTranslator::getVectorMsgFromVector(Vector vector) {
    VectorMsg vector_msg;
    vector_msg.set_x(vector.x());
    vector_msg.set_y(vector.y());
    return vector_msg;
}
