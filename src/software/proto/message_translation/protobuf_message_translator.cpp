#include "software/proto/protobuf_primitive_visitor.h"

std::unique_ptr<VisionMsg> ProtobufMessageTranslator::getVisionMsgFromWorld(World world)
{
}

std::unique_ptr<PrimitiveMsg> ProtoMessageTranslator::getPrimitiveMsgFromPrimitiveVector(
    ConstPrimitiveVectorPtr primitives)
{
    PrimitiveMsg primitive_msg;
}

std::unique_ptr<PointMsg> ProtoMessageTranslator::getPointMsgFromPoint(Point point)
{
    auto point_msg = std::make_unique<PointMsg>();
    point_msg->set_x(point.x());
    point_msg->set_y(point.y());
    return point_msg;
}

std::unique_ptr<AngleMsg> ProtoMessageTranslator::getAngleMsgFromAngle(Angle angle)
{
    auto angle_msg = std::make_unique<AngleMsg>();
    angle_msg->set_radians(angle.toRadians());
    return angle_msg;
}

std::unique_ptr<VectorMsg> ProtoMessageTranslator::getVectorMsgFromVector(Vector vector)
{
    auto vector_msg = std::make_unique<VectorMsg>();
    vector_msg->set_x(vector.x());
    vector_msg->set_y(vector.y());
    return vector_msg;
}

std::unique_ptr<RadioPrimitiveMsg>
ProtoMessageTranslator::getRadioPrimitiveMsgFromPrimitive(Primitive primitive)
{
    auto primitive_visitor = ProtobufPrimitiveVisitor();
    primitive_visitor.visit(primitive);
    return primitive_visitor.getRadioPrimitiveMsg();
}
