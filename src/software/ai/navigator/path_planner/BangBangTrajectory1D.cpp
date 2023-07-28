#include "BangBangTrajectory1D.h"

BangBangTrajectory1D::BangBangTrajectory1D(double initial_pos, double final_pos, double initial_vel,
                                           double final_vel, double max_vel, double max_accel, double max_decel)
                                           : trajectory_parts(3)
{
    // Unify the signs of the inputs
    double max_accel = std::abs(max_accel);
    double max_decel = std::abs(max_decel);
    double max_vel = std::abs(max_vel);

    // The closest position to reach the final velocity
    double a;
    if (std::signbit(final_vel - initial_vel) > 0)
    {
        // We need to accelerate
        a = max_accel;
    }
    else
    {
        // We need to decelerate
        a = -max_decel;
    }

    // The minimum  position to reach the final velocity
    // Based on: vf^2 = vi^2 + 2ad
    // TODO: What happens if the initial and final vel is positive, but final pos is behind us?
    //       We have to slow down, then speed up... This just goes to final vel without considering
    //       the final pos and where it is relative to initial pos
    double min_dist_to_final_vel = (std::pow(final_vel, 2) - std::pow(initial_vel, 2)) / (2 * a);

    if (closest_pos_at_final_vel <= final_pos)
    {

    }
    else
    {
        // We can't decelerate fast enough to
    }
}

double BangBangTrajectory1D::getPosition(const Duration t)
{
    return 0;
}

double BangBangTrajectory1D::getVelocity(const Duration t)
{
    return 0;
}

double BangBangTrajectory1D::getAcceleration(const Duration t)
{
    return 0;
}

Duration BangBangTrajectory1D::getTotalTime()
{
    return Duration();
}

inline double BangBangTrajectory1D::closestPositionToReachVelocity(double initial_position, double initial_velocity, double max_deceleration)
{
}
