#include "software/ai/intent/move_intent.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority,
                       DribblerEnable enable_dribbler, MoveType move_type,
                       AutochickType autokick, BallCollisionType ball_collision_type)
    : MoveIntent(MovePrimitive(robot_id, dest, final_angle, final_speed, enable_dribbler,
                               move_type, autokick),
                 priority, ball_collision_type)
{
}

MoveIntent::MoveIntent(MovePrimitive move_primitive, unsigned int priority,
                       BallCollisionType ball_collision_type)
    : MovePrimitive(move_primitive),
      Intent(priority),
      ball_collision_type(ball_collision_type)
{
    Intent::updateNavigatorParams(getRobotId(), getDestination(), getFinalAngle(),
                                  getFinalSpeed(), getBallCollisionType());
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

std::optional<std::unique_ptr<Intent>> MoveIntent::createWithNewFinalSpeedAndDestination(
    Point destination, double final_speed) const
{
    // MovePrimitive::updateFinalSpeedAndDestination(destination, final_speed);
    MovePrimitive move_primitive(*this);
    move_primitive.updateFinalSpeedAndDestination(destination, final_speed);
    return std::make_unique<MoveIntent>(move_primitive, getPriority(),
                                        getBallCollisionType());
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return MovePrimitive::operator==(other) && Intent::operator==(other);
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
