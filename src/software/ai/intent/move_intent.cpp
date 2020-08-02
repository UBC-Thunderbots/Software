#include "software/ai/intent/move_intent.h"

#include "shared/constants.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority,
                       DribblerEnable enable_dribbler, MoveType move_type,
                       AutochickType autokick, BallCollisionType ball_collision_type)
    : NavigatingIntent(robot_id,
                       ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
                           MovePrimitive(robot_id, dest, final_angle, final_speed,
                                         enable_dribbler, move_type, autokick)),
                       priority, dest, final_angle, final_speed, ball_collision_type)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}
