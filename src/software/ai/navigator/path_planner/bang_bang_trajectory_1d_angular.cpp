#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"

void BangBangTrajectory1DAngular::generate(Angle initial_orient, Angle final_orient, AngularVelocity initial_angular_vel,
                                           AngularVelocity max_angular_vel, AngularAcceleration max_angular_accel,
                                           AngularAcceleration max_angular_decel)
{
    Angle relative_final_orient = initial_orient + (final_orient - initial_orient).clamp();
    trajectory.generate(initial_orient.toRadians(), relative_final_orient.toRadians(), initial_angular_vel.toRadians(),
                        max_angular_vel.toRadians(), max_angular_accel.toRadians(),
                        max_angular_decel.toRadians());
}

Angle BangBangTrajectory1DAngular::getPosition(Duration t) const
{
    return Angle::fromRadians(trajectory.getPosition(t));
}

AngularVelocity BangBangTrajectory1DAngular::getVelocity(Duration t) const
{
    return AngularVelocity::fromRadians(trajectory.getVelocity(t));
}

AngularAcceleration BangBangTrajectory1DAngular::getAcceleration(Duration t) const
{
    return AngularAcceleration::fromRadians(trajectory.getAcceleration(t));
}

Duration BangBangTrajectory1DAngular::getTotalTime() const
{
    return trajectory.getTotalTime();
}

const std::vector<BangBangTrajectory1D::TrajectoryPart> &BangBangTrajectory1DAngular::getTrajectoryParts() const
{
    return trajectory.getTrajectoryParts();
}
