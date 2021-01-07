#include "software/ai/intent/autochip_move_intent.h"

AutochipMoveIntent::AutochipMoveIntent(unsigned int robot_id, const Point &destination,
                                       const Angle &final_angle, double final_speed,
                                       DribblerMode dribbler_mode,
                                       double chip_distance_meters,
                                       BallCollisionType ball_collision_type)
    : MoveIntent(robot_id, destination, final_angle, final_speed, dribbler_mode,
                 ball_collision_type),
      chip_distance_meters(chip_distance_meters)
{
}

AutochipMoveIntent::AutochipMoveIntent(MoveIntent move_intent,
                                       double chip_distance_meters)
    : MoveIntent(move_intent), chip_distance_meters(chip_distance_meters)
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

double AutochipMoveIntent::getChipDistance() const
{
    return chip_distance_meters;
}

bool AutochipMoveIntent::operator==(const AutochipMoveIntent &other) const
{
    return MoveIntent::operator==(other) &&
           this->chip_distance_meters == other.chip_distance_meters;
}

bool AutochipMoveIntent::operator!=(const AutochipMoveIntent &other) const
{
    return !((*this) == other);
}
