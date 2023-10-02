#include "software/ai/navigator/path_planner/kinematic_constraints.h"

KinematicConstraints::KinematicConstraints(double max_velocity, double max_acceleration,
                                           double max_deceleration)
    : max_velocity(max_velocity),
      max_acceleration(max_acceleration),
      max_deceleration(max_deceleration)
{
}

double KinematicConstraints::getMaxVelocity() const
{
    return max_velocity;
}

double KinematicConstraints::getMaxAcceleration() const
{
    return max_acceleration;
}

double KinematicConstraints::getMaxDeceleration() const
{
    return max_deceleration;
}
