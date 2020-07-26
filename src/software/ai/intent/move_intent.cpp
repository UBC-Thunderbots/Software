#include "software/ai/intent/move_intent.h"

#include "shared/constants.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority,
                       DribblerEnable enable_dribbler, MoveType move_type,
                       AutochickType autokick, BallCollisionType ball_collision_type)
    : Intent(robot_id,
             ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
                 MovePrimitive(robot_id, dest, final_angle, final_speed, enable_dribbler,
                               move_type, autokick)),
             priority),
      ball_collision_type(ball_collision_type)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

BallCollisionType MoveIntent::getBallCollisionType() const
{
    return ball_collision_type;
}

void MoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

PrimitiveMsg MoveIntent::getPrimitiveMsg(Point destination, double final_speed) const
{
    PrimitiveMsg new_primitive_msg = Intent::getPrimitiveMsg();

    PrimitiveParamsMsg new_primitive_params_msg = new_primitive_msg.move();
    new_primitive_params_msg.set_parameter1(
        static_cast<float>(destination.x() * MILLIMETERS_PER_METER));
    new_primitive_params_msg.set_parameter2(
        static_cast<float>(destination.y() * MILLIMETERS_PER_METER));
    new_primitive_params_msg.set_parameter4(
        static_cast<float>(final_speed * MILLIMETERS_PER_METER));

    *(new_primitive_msg.mutable_move()) = new_primitive_params_msg;

    return new_primitive_msg;
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return Intent::operator==(other);
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
