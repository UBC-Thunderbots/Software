#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"

#include <cmath>

#include "software/geom/algorithms/is_in_range.h"
#include "software/logger/logger.h"

BangBangTrajectory1D::BangBangTrajectory1D()
{
    // Add a default trajectory part to avoid segfaults
    // when the getters are called before generate
    addTrajectoryPart(TrajectoryPart());
}

void BangBangTrajectory1D::generate(double initial_pos, double final_pos,
                                    double initial_vel, double max_vel, double max_accel,
                                    double max_decel)
{
    // We will overwrite the previous trajectory
    num_trajectory_parts = 0;

    CHECK(max_vel != 0) << "Max velocity cannot be 0";
    CHECK(max_accel != 0) << "Max acceleration cannot be 0";
    CHECK(max_decel != 0) << "Max deceleration cannot be 0";

    // Unify the signs of the kinematic constraints
    max_accel = std::abs(max_accel);
    max_decel = std::abs(max_decel);
    max_vel   = std::abs(max_vel);

    // From initial position, where the closest position is where we can stop.
    // If it is not between initial and final position, then we must break right away.
    double stop_pos = closestPositionToStop(initial_pos, initial_vel, max_decel);
    if (isInRangeInclusive(stop_pos, initial_pos, final_pos))
    {
        // If we start decelerating right now, we will stop before our destination,
        // so we can accelerate for a bit before having to decelerate.
        double direction      = std::copysign(1, final_pos - initial_pos);
        double triangular_pos = triangularProfileStopPosition(
            initial_pos, initial_vel, max_vel, max_accel, max_decel, direction);
        if (isInRangeExclusive(triangular_pos, initial_pos, final_pos))
        {
            // We have time to reach max velocity, so we can use a trapezoidal profile
            generateTrapezoidalTrajectory(initial_pos, final_pos, initial_vel, max_vel,
                                          max_accel, max_decel);
        }
        else
        {
            // We can't reach max velocity and cruise at it, so we have to use
            // a triangular profile
            generateTriangularTrajectory(initial_pos, final_pos, initial_vel, max_accel,
                                         max_decel);
        }
    }
    else
    {
        // Either there is no way for us not to overshoot the destination, or the initial
        // velocity is moving away from the destination. In either case, we have to
        // decelerate to stop first, then we can generate a profile to our destination.
        // vf = vi + at  =>  t = -vi / t  (vf = 0)
        double time_to_stop_sec = std::abs(initial_vel) / max_decel;
        addTrajectoryPart({.end_time_sec = time_to_stop_sec,
                           .position     = initial_pos,
                           .velocity     = initial_vel,
                           .acceleration = -std::copysign(max_decel, initial_vel)});

        double direction      = std::copysign(1, final_pos - stop_pos);
        double triangular_pos = triangularProfileStopPosition(
            stop_pos, 0, max_vel, max_accel, max_decel, direction);
        if (isInRangeExclusive(triangular_pos, stop_pos, final_pos))
        {
            // We have time to reach max velocity, so we can use a trapezoidal profile
            generateTrapezoidalTrajectory(stop_pos, final_pos, 0, max_vel, max_accel,
                                          max_decel, time_to_stop_sec);
        }
        else
        {
            // We can't reach max velocity and cruise at it, so we have to use
            // a triangular profile
            generateTriangularTrajectory(stop_pos, final_pos, 0, max_accel, max_decel,
                                         time_to_stop_sec);
        }
    }
}

void BangBangTrajectory1D::generateTrapezoidalTrajectory(
    double initial_pos, double final_pos, double initial_vel, double max_vel,
    double max_accel, double max_decel, double time_offset_sec)
{
    if (final_pos < initial_pos)
    {
        max_vel   = -max_vel;
        max_accel = -max_accel;
        max_decel = -max_decel;
    }

    // Calculate time and distance to accelerate to max velocity
    // Note that initial velocity may be higher than max velocity,
    // so we might need to decelerate to reach max velocity.
    double initial_accel;
    if (std::abs(initial_vel) < std::abs(max_vel))
    {
        initial_accel = max_accel;
    }
    else
    {
        initial_accel = -max_decel;
    }
    double t1 = std::abs((max_vel - initial_vel) / initial_accel);
    double d1 = initial_vel * t1 + 0.5 * initial_accel * t1 * t1;

    // Calculate time and distance to decelerate from max velocity to 0
    double t3 = max_vel / max_decel;
    double d3 = max_vel * t3 + 0.5 * -max_decel * t3 * t3;

    // Calculate distance and time we can travel at max velocity
    double d2 = (final_pos - initial_pos) - (d1 + d3);
    double t2 = d2 / max_vel;

    // Add the trajectory parts
    addTrajectoryPart({.end_time_sec = time_offset_sec + t1,
                       .position     = initial_pos,
                       .velocity     = initial_vel,
                       .acceleration = initial_accel});
    addTrajectoryPart({.end_time_sec = time_offset_sec + t1 + t2,
                       .position     = initial_pos + d1,
                       .velocity     = max_vel,
                       .acceleration = 0});
    addTrajectoryPart({.end_time_sec = time_offset_sec + t1 + t2 + t3,
                       .position     = initial_pos + d1 + d2,
                       .velocity     = max_vel,
                       .acceleration = -max_decel});
}

void BangBangTrajectory1D::generateTriangularTrajectory(
    double initial_pos, double final_pos, double initial_vel, double max_accel,
    double max_decel, double time_offset_sec)
{
    double direction = std::copysign(1.0, final_pos - initial_pos);
    double dist      = std::abs(final_pos - initial_pos);

    // Given the following constraints:
    // - Travel exactly a distance of dist
    // - Have a final velocity of 0
    // - Take the least amount of time possible (i.e. accelerate until we have to
    // decelerate) Calculate the time it takes for us to decelerate from the highest
    // velocity we can reach to a full stop. The full derivation of the formula can be
    // found here: https://www.desmos.com/calculator/qvrvtplgk7 Note that the full
    // derivation also supports a non-zero final velocity, but we currently don't support
    // that here.
    double t_decel = std::sqrt((initial_vel * initial_vel + 2 * dist * max_accel) /
                               (max_decel * (max_accel + max_decel)));

    double signed_accel = std::copysign(max_accel, direction);
    double signed_decel = -std::copysign(max_decel, direction);

    // Calculate the max velocity we will reach
    // vf = vi + at  =>  vi = -at  (vf = 0)
    double v_max_reached = -signed_decel * t_decel;

    // Calculate the time to accelerate to the max reached velocity
    double t_accel = (v_max_reached - initial_vel) / signed_accel;
    double d_accel = initial_vel * t_accel + 0.5 * signed_accel * t_accel * t_accel;

    // Add the trajectory parts
    addTrajectoryPart({.end_time_sec = time_offset_sec + t_accel,
                       .position     = initial_pos,
                       .velocity     = initial_vel,
                       .acceleration = signed_accel});
    addTrajectoryPart({.end_time_sec = time_offset_sec + t_accel + t_decel,
                       .position     = initial_pos + d_accel,
                       .velocity     = v_max_reached,
                       .acceleration = signed_decel});
}

double BangBangTrajectory1D::getPosition(double t_sec) const
{
    TrajectoryPart traj_part;
    double t_delta_sec;
    getTrajPartAndDeltaTime(t_sec, traj_part, t_delta_sec);

    // d = vi * t + 0.5 * a * t^2
    // p = pi + d
    return traj_part.position + traj_part.velocity * t_delta_sec +
           0.5 * traj_part.acceleration * t_delta_sec * t_delta_sec;
}

double BangBangTrajectory1D::getVelocity(double t_sec) const
{
    TrajectoryPart traj_part;
    double t_delta_sec;
    getTrajPartAndDeltaTime(t_sec, traj_part, t_delta_sec);

    // vf = vi + at
    return traj_part.velocity + traj_part.acceleration * t_delta_sec;
}

double BangBangTrajectory1D::getAcceleration(double t_sec) const
{
    t_sec                   = std::clamp(t_sec, 0.0, getTotalTime());
    size_t trajectory_index = getTrajectoryIndexAtTime(t_sec);
    return trajectory_parts[trajectory_index].acceleration;
}

void BangBangTrajectory1D::getTrajPartAndDeltaTime(
    double t_sec, BangBangTrajectory1D::TrajectoryPart &out_traj_part,
    double &out_t_delta_sec) const
{
    t_sec                   = std::clamp(t_sec, 0.0, getTotalTime());
    size_t trajectory_index = getTrajectoryIndexAtTime(t_sec);
    out_traj_part           = trajectory_parts[trajectory_index];

    double last_part_start_time = 0.0;
    if (trajectory_index > 0)
    {
        last_part_start_time = trajectory_parts[trajectory_index - 1].end_time_sec;
    }
    out_t_delta_sec = t_sec - last_part_start_time;
}

inline double BangBangTrajectory1D::closestPositionToStop(double initial_pos,
                                                          double initial_vel,
                                                          double max_decel) const
{
    // vf^2 = vi^2 + 2ad  =>  d = (vf^2 - vi^2) / 2a  =>  d = (vi^2) / 2a
    double dist_to_stop = (initial_vel * initial_vel) / (2 * std::abs(max_decel));

    // Make the distance to stop negative if we are moving backwards
    return initial_pos + std::copysign(dist_to_stop, initial_vel);
}

inline double BangBangTrajectory1D::triangularProfileStopPosition(
    double initial_pos, double initial_vel, double max_vel, double max_accel,
    double max_decel, double direction) const
{
    if (std::abs(initial_vel) <= max_vel)
    {
        // Distance to accelerate to max velocity
        double d_accel =
            (max_vel * max_vel - initial_vel * initial_vel) / (2 * max_accel);

        // Distance to decelerate from max velocity to 0
        double d_decel = (max_vel * max_vel) / (2 * max_decel);

        return initial_pos + std::copysign(d_accel + d_decel, direction);
    }
    else
    {
        // Initial velocity is higher than max velocity, so we will just decelerate to 0
        double d_decel = (initial_vel * initial_vel) / (2 * max_decel);
        return initial_pos + std::copysign(d_decel, direction);
    }
}

size_t BangBangTrajectory1D::getTrajectoryIndexAtTime(double t_sec) const
{
    for (unsigned int i = 0; i < num_trajectory_parts; i++)
    {
        if (t_sec <= trajectory_parts[i].end_time_sec)
        {
            return i;
        }
    }

    return num_trajectory_parts - 1;
}

std::pair<double, double> BangBangTrajectory1D::getMinMaxPositions() const
{
    // The min and max positions of the overall trajectory should be at
    // the position which one of the trajectory parts is at, since we
    // add a new trajectory part whenever the direction of movement changes.
    // In other words, we have a trajectory part for every time the velocity
    // is 0.
    std::pair<double, double> min_max_pos = {std::numeric_limits<double>::max(),
                                             std::numeric_limits<double>::min()};
    for (const TrajectoryPart &part : trajectory_parts)
    {
        min_max_pos.first  = std::min(min_max_pos.first, part.position);
        min_max_pos.second = std::max(min_max_pos.second, part.position);
    }

    double final_position = getDestination();
    min_max_pos.first     = std::min(min_max_pos.first, final_position);
    min_max_pos.second    = std::max(min_max_pos.second, final_position);

    return min_max_pos;
}

inline void BangBangTrajectory1D::addTrajectoryPart(
    const BangBangTrajectory1D::TrajectoryPart &part)
{
    CHECK(num_trajectory_parts < MAX_TRAJECTORY_PARTS)
        << "BangBangTrajectory1D::addTrajectoryPart was called when the trajectory_parts array was full";
    trajectory_parts[num_trajectory_parts++] = part;
}

size_t BangBangTrajectory1D::getNumTrajectoryParts() const
{
    return num_trajectory_parts;
}

const BangBangTrajectory1D::TrajectoryPart &BangBangTrajectory1D::getTrajectoryPart(
    size_t index) const
{
    CHECK(index < num_trajectory_parts)
        << "BangBangTrajectory1D::getTrajectoryPart was called with index " << index
        << " which is out of bounds of trajectory size " << num_trajectory_parts;
    return trajectory_parts[index];
}
