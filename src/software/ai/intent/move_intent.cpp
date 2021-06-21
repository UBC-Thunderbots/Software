#include "software/ai/intent/move_intent.h"

MoveIntent::MoveIntent(unsigned int robot_id, const Point& destination,
                       const Angle& final_angle, double final_speed,
                       const DribblerMode& dribbler_mode,
                       const BallCollisionType& ball_collision_type,
                       const AutoChipOrKick& auto_chip_or_kick,
                       const MaxAllowedSpeedMode& max_allowed_speed_mode,
                       double target_spin_rev_per_s,
                       const RobotConstants_t& robot_constants)
    : NavigatingIntent(robot_id, destination, final_speed, ball_collision_type,
                       robot_constants, max_allowed_speed_mode),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      auto_chip_or_kick(auto_chip_or_kick),
      target_spin_rev_per_s(target_spin_rev_per_s)
{
}

void MoveIntent::accept(IntentVisitor& visitor) const
{
    visitor.visit(*this);
}

void MoveIntent::accept(NavigatingIntentVisitor& visitor) const
{
    visitor.visit(*this);
}

const Angle& MoveIntent::getFinalAngle() const
{
    return final_angle;
}

const DribblerMode& MoveIntent::getDribblerMode() const
{
    return dribbler_mode;
}

const AutoChipOrKick& MoveIntent::getAutoChipOrKick() const
{
    return auto_chip_or_kick;
}

double MoveIntent::getTargetSpinRevPerS() const
{
    return target_spin_rev_per_s;
}

bool MoveIntent::operator==(const MoveIntent& other) const
{
    return NavigatingIntent::operator==(other) &&
           this->final_angle == other.final_angle &&
           this->dribbler_mode == other.dribbler_mode &&
           this->auto_chip_or_kick == other.auto_chip_or_kick &&
           this->target_spin_rev_per_s == other.target_spin_rev_per_s;
}

bool MoveIntent::operator!=(const MoveIntent& other) const
{
    return !((*this) == other);
}
