#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"

#include <cmath>

#include "software/geom/algorithms/is_in_range.h"
#include "software/logger/logger.h"

BangBangTrajectory1D::BangBangTrajectory1D()
{
    // Add a default trajectory part to avoid segfaults
    // when the getters are called before generate
    trajectory_parts.push_back(TrajectoryPart());
}

void BangBangTrajectory1D::generate(double initial_pos, double final_pos,
                                    double initial_vel, double max_vel, double max_accel,
                                    double max_decel)
{
//    static unsigned long int run = 0;
//    if (run++ % 10000 == 0)
//    {
//        std::cout << run << std::endl;
//    }
    trajectory_parts.clear();

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
        double direction = std::copysign(1, final_pos - initial_pos);
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
        Duration time_to_stop = Duration::fromSeconds(std::abs(initial_vel) / max_decel);
        trajectory_parts.push_back({.end_time     = time_to_stop,
                                    .position     = initial_pos,
                                    .velocity     = initial_vel,
                                    .acceleration = -std::copysign(max_decel, initial_vel)});

        double direction = std::copysign(1, final_pos - stop_pos);
        double triangular_pos =
                triangularProfileStopPosition(stop_pos, 0, max_vel, max_accel, max_decel, direction);
        if (isInRangeExclusive(triangular_pos, stop_pos, final_pos))
        {
            // We have time to reach max velocity, so we can use a trapezoidal profile
            generateTrapezoidalTrajectory(stop_pos, final_pos, 0, max_vel, max_accel,
                                          max_decel, time_to_stop);
        }
        else
        {
            // We can't reach max velocity and cruise at it, so we have to use
            // a triangular profile
            generateTriangularTrajectory(stop_pos, final_pos, 0, max_accel, max_decel,
                                         time_to_stop);
        }
    }
}

void BangBangTrajectory1D::generateTrapezoidalTrajectory(
    double initial_pos, double final_pos, double initial_vel, double max_vel,
    double max_accel, double max_decel, Duration time_offset)
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
    double d1 = initial_vel * t1 + 0.5 * initial_accel * std::pow(t1, 2.0);

    // Calculate time and distance to decelerate from max velocity to 0
    double t3 = max_vel / max_decel;
    double d3 = max_vel * t3 + 0.5 * -max_decel * std::pow(t3, 2.0);

    // Calculate distance and time we can travel at max velocity
    double d2 = (final_pos - initial_pos) - (d1 + d3);
    double t2 = d2 / max_vel;

    // Add the trajectory parts
    trajectory_parts.insert(
        trajectory_parts.end(),
        {{.end_time     = time_offset + Duration::fromSeconds(t1),
          .position     = initial_pos,
          .velocity     = initial_vel,
          .acceleration = initial_accel},
         {.end_time     = time_offset + Duration::fromSeconds(t1 + t2),
          .position     = initial_pos + d1,
          .velocity     = max_vel,
          .acceleration = 0},
         {.end_time     = time_offset + Duration::fromSeconds(t1 + t2 + t3),
          .position     = initial_pos + d1 + d2,
          .velocity     = max_vel,
          .acceleration = -max_decel}});
}

void BangBangTrajectory1D::generateTriangularTrajectory(
    double initial_pos, double final_pos, double initial_vel, double max_accel,
    double max_decel, Duration time_offset)
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
    double t_decel = std::sqrt((std::pow(initial_vel, 2) + 2 * dist * max_accel) /
                               (max_decel * (max_accel + max_decel)));

    double signed_accel = std::copysign(max_accel, direction);
    double signed_decel = -std::copysign(max_decel, direction);

    // Calculate the max velocity we will reach
    // vf = vi + at  =>  vi = -at  (vf = 0)
    double v_max_reached = -signed_decel * t_decel;

    // Calculate the time to accelerate to the max reached velocity
    double t_accel = (v_max_reached - initial_vel) / signed_accel;
    double d_accel = initial_vel * t_accel + 0.5 * signed_accel * std::pow(t_accel, 2.0);

    // Add the trajectory parts
    trajectory_parts.insert(
        trajectory_parts.end(),
        {{.end_time     = time_offset + Duration::fromSeconds(t_accel),
          .position     = initial_pos,
          .velocity     = initial_vel,
          .acceleration = signed_accel},
         {.end_time     = time_offset + Duration::fromSeconds(t_accel + t_decel),
          .position     = initial_pos + d_accel,
          .velocity     = v_max_reached,
          .acceleration = signed_decel}});
}

double BangBangTrajectory1D::getPosition(Duration t) const
{
    TrajectoryPart traj_part;
    Duration t_delta;
    getTrajPartAndDeltaTime(t, traj_part, t_delta);

    return traj_part.position + traj_part.velocity * t_delta.toSeconds() +
           0.5 * traj_part.acceleration * std::pow(t_delta.toSeconds(), 2.0);
}

double BangBangTrajectory1D::getVelocity(Duration t) const
{
    TrajectoryPart traj_part;
    Duration t_delta;
    getTrajPartAndDeltaTime(t, traj_part, t_delta);

    return traj_part.velocity + traj_part.acceleration * t_delta.toSeconds();
}

double BangBangTrajectory1D::getAcceleration(Duration t) const
{
    t                       = std::clamp(t, Duration::fromSeconds(0), getTotalTime());
    size_t trajectory_index = getTrajectoryIndexAtTime(t);
    return trajectory_parts[trajectory_index].acceleration;
}

void BangBangTrajectory1D::getTrajPartAndDeltaTime(
    Duration t, BangBangTrajectory1D::TrajectoryPart &out_traj_part,
    Duration &out_t_delta) const
{
    t                       = std::clamp(t, Duration::fromSeconds(0), getTotalTime());
    size_t trajectory_index = getTrajectoryIndexAtTime(t);
    out_traj_part           = trajectory_parts[trajectory_index];

    Duration last_part_start_time;
    if (trajectory_index > 0)
    {
        last_part_start_time = trajectory_parts[trajectory_index - 1].end_time;
    }
    out_t_delta = t - last_part_start_time;
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

inline double
BangBangTrajectory1D::triangularProfileStopPosition(double initial_pos, double initial_vel, double max_vel,
                                                    double max_accel, double max_decel, double direction) const
{
    if (std::abs(initial_vel) <= max_vel)
    {
        // Distance to accelerate to max velocity
        double d_accel = (std::pow(max_vel, 2.0) - std::pow(initial_vel, 2.0)) /
                         (2 * max_accel);

        // Distance to decelerate from max velocity to 0
        double d_decel = std::pow(max_vel, 2.0) / (2 * max_decel);

        return initial_pos + std::copysign(d_accel + d_decel, direction);
    }
    else
    {
        // Initial velocity is higher than max velocity, so we will just decelerate to 0
        double d_decel = std::pow(initial_vel, 2.0) / (2 * max_decel);
        return initial_pos + std::copysign(d_decel, direction);
    }
}

const std::vector<BangBangTrajectory1D::TrajectoryPart>
    &BangBangTrajectory1D::getTrajectoryParts() const
{
    return trajectory_parts;
}

size_t BangBangTrajectory1D::getTrajectoryIndexAtTime(Duration t) const
{
    for (unsigned int i = 0; i < trajectory_parts.size(); i++)
    {
        if (t <= trajectory_parts[i].end_time)
        {
            return i;
        }
    }

    return trajectory_parts.size() - 1;
}

std::pair<double, double> BangBangTrajectory1D::getMinMaxPositions() const
{
    std::pair<double, double> min_max_pos = {std::numeric_limits<double>::max(),
                                             std::numeric_limits<double>::min()};
    for (const TrajectoryPart& part : trajectory_parts)
    {
        min_max_pos.first  = std::min(min_max_pos.first, part.position);
        min_max_pos.second = std::max(min_max_pos.second, part.position);
    }
    return min_max_pos;
}
