#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"

void BangBangTrajectory1DAngular::generate(Angle initial_orient, Angle final_orient,
                                           AngularVelocity initial_angular_vel,
                                           AngularVelocity max_angular_vel,
                                           AngularAcceleration max_angular_accel,
                                           AngularAcceleration max_angular_decel)
{
    // We use the relative final orientation as the destination of the trajectory
    // to avoid issues with the discontinuity at 2PI == 0 or -PI == PI.
    Angle relative_final_orient =
        initial_orient + (final_orient - initial_orient).clamp();
    trajectory.generate(initial_orient.toRadians(), relative_final_orient.toRadians(),
                        initial_angular_vel.toRadians(), max_angular_vel.toRadians(),
                        max_angular_accel.toRadians(), max_angular_decel.toRadians());
}

Angle BangBangTrajectory1DAngular::getPosition(double t_sec) const
{
    return Angle::fromRadians(trajectory.getPosition(t_sec));
}

AngularVelocity BangBangTrajectory1DAngular::getVelocity(double t_sec) const
{
    return AngularVelocity::fromRadians(trajectory.getVelocity(t_sec));
}

AngularAcceleration BangBangTrajectory1DAngular::getAcceleration(double t_sec) const
{
    return AngularAcceleration::fromRadians(trajectory.getAcceleration(t_sec));
}

double BangBangTrajectory1DAngular::getTotalTime() const
{
    return trajectory.getTotalTime();
}
