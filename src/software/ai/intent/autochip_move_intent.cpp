#include "software/ai/intent/autochip_move_intent.h"

AutochipMoveIntent::AutochipMoveIntent(unsigned int robot_id, const Point &destination,
                                       const Angle &final_angle, double final_speed,
                                       DribblerMode dribbler_mode,
                                       double chip_distance_meters,
                                       BallCollisionType ball_collision_type)
    : NavigatingIntent(robot_id, destination, final_speed, ball_collision_type),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      chip_distance_meters(chip_distance_meters)
{
}

void AutochipMoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

void AutochipMoveIntent::accept(NavigatingIntentVisitor &visitor) const
{
    visitor.visit(*this);
}

const Angle &AutochipMoveIntent::getFinalAngle() const
{
    return final_angle;
}

const DribblerMode &AutochipMoveIntent::getDribblerMode() const
{
    return dribbler_mode;
}

double AutochipMoveIntent::getChipDistance() const
{
    return chip_distance_meters;
}

bool AutochipMoveIntent::operator==(const AutochipMoveIntent &other) const
{
    return NavigatingIntent::operator==(other) &&
           this->final_angle == other.final_angle &&
           this->chip_distance_meters == other.chip_distance_meters &&
           this->dribbler_mode == other.dribbler_mode;
}

bool AutochipMoveIntent::operator!=(const AutochipMoveIntent &other) const
{
    return !((*this) == other);
}
