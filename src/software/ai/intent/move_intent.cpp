#include "software/ai/intent/move_intent.h"

MoveIntent::MoveIntent(unsigned int robot_id, const Point &destination,
                       const Angle &final_angle, double final_speed,
                       DribblerMode dribbler_mode, BallCollisionType ball_collision_type,
                       std::optional<TbotsProto::Autochipkick> autochipkick,
                       MaxAllowedSpeedMode max_allowed_speed_mode)
    : NavigatingIntent(robot_id, destination, final_speed, ball_collision_type,
                       max_speed_m_per_s),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      autochipkick(autochipkick)
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

std::optional<TbotsProto::Autochipkick> MoveIntent::getAutochipkick() const
{
    return autochipkick;
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
