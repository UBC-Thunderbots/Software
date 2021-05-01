#include "software/ai/intent/move_intent.h"

MoveIntent::MoveIntent(unsigned int robot_id, const Point &destination,
                       const Angle &final_angle, double final_speed,
                       DribblerMode dribbler_mode, BallCollisionType ball_collision_type,
                       AutoChipOrKick auto_chip_or_kick,
                       MaxAllowedSpeedMode max_allowed_speed_mode)
    : NavigatingIntent(robot_id, destination, final_speed, ball_collision_type,
                       max_allowed_speed_mode),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      auto_chip_or_kick(auto_chip_or_kick)
{
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

const DribblerMode &MoveIntent::getDribblerMode() const
{
    return dribbler_mode;
}

AutoChipOrKick MoveIntent::getAutoChipOrKick() const
{
    return auto_chip_or_kick;
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return NavigatingIntent::operator==(other) &&
           this->final_angle == other.final_angle &&
           this->dribbler_mode == other.dribbler_mode;
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
