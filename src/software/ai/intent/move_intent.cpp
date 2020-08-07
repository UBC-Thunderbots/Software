#include "software/ai/intent/move_intent.h"

#include "shared/constants.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &destination,
                       const Angle &final_angle, double final_speed,
                       unsigned int priority, DribblerEnable enable_dribbler,
                       MoveType move_type, AutochickType autokick,
                       BallCollisionType ball_collision_type)
    : Intent(robot_id, priority, ball_collision_type, destination),
      destination(destination),
      final_angle(final_angle),
      final_speed(final_speed),
      enable_dribbler(enable_dribbler),
      move_type(move_type),
      autokick(autokick)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

Point MoveIntent::getDestination() const
{
    return destination;
}

Angle MoveIntent::getFinalAngle() const
{
    return final_angle;
}

double MoveIntent::getFinalSpeed() const
{
    return final_speed;
}

AutochickType MoveIntent::getAutochickType() const
{
    return autokick;
}

DribblerEnable MoveIntent::getDribblerEnable() const
{
    return enable_dribbler;
}

MoveType MoveIntent::getMoveType() const
{
    return move_type;
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return Intent::operator==(other) && this->destination == other.destination &&
           this->final_angle == other.final_angle &&
           this->final_speed == other.final_speed &&
           this->enable_dribbler == other.enable_dribbler &&
           this->move_type == other.move_type && this->autokick == other.autokick;
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
