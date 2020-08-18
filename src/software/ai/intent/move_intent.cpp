#include "software/ai/intent/move_intent.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &destination,
                       const Angle &final_angle, double final_speed,
                       unsigned int priority, DribblerEnable enable_dribbler,
                       MoveType move_type, AutochickType autokick,
                       BallCollisionType ball_collision_type)
    : NavigatingIntent(robot_id, priority, destination, final_speed, ball_collision_type),
      final_angle(final_angle),
      enable_dribbler(enable_dribbler),
      move_type(move_type),
      autokick(autokick)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void MoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

void MoveIntent::accept(NavigatingIntentVisitor &visitor) const
{
    visitor.visit(*this);
}

const Angle &MoveIntent::getFinalAngle() const
{
    return final_angle;
}

const AutochickType &MoveIntent::getAutochickType() const
{
    return autokick;
}

const DribblerEnable &MoveIntent::getDribblerEnable() const
{
    return enable_dribbler;
}

const MoveType &MoveIntent::getMoveType() const
{
    return move_type;
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return NavigatingIntent::operator==(other) &&
           this->final_angle == other.final_angle &&
           this->enable_dribbler == other.enable_dribbler &&
           this->move_type == other.move_type && this->autokick == other.autokick;
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
