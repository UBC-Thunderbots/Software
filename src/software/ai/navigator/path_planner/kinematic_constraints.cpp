#include "software/ai/navigator/path_planner/kinematic_constraints.h"

KinematicConstraints::KinematicConstraints(double max_velocity, double max_acceleration, double max_deceleration) :
    max_velocity(max_velocity), max_acceleration(max_acceleration), max_deceleration(max_deceleration)
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

void KinematicConstraints::setMaxVelocity(const double new_max_velocity)
{
    max_velocity = new_max_velocity;
}

void KinematicConstraints::setMaxAcceleration(const double new_max_acceleration)
{
    max_acceleration = new_max_acceleration;
}

void KinematicConstraints::setMaxDeceleration(const double new_max_deceleration)
{
    max_deceleration = new_max_deceleration;
}
