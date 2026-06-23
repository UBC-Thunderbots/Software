#include "software/ai/navigator/trajectory/jerk_limited_trajectory_1d.h"

#include <algorithm>
#include <cmath>

#include "software/geom/algorithms/is_in_range.h"

JerkLimitedTrajectory1D::JerkLimitedTrajectory1D()
{
    addTrajectoryPart(TrajectoryPart());
}

void JerkLimitedTrajectory1D::generate(const double initial_pos, const double final_pos,
                                       const double initial_vel,
                                       const double initial_accel, double max_vel,
                                       double max_accel, double max_decel,
                                       double max_jerk)
{
    trajectory_parts.clear();

    max_accel = std::abs(max_accel);
    max_decel = std::abs(max_decel);
    max_vel   = std::abs(max_vel);
    max_jerk  = std::abs(max_jerk);

    const double distance = final_pos - initial_pos;

    if (std::abs(distance) < EPSILON)
    {
        // Already at target, bring the robot to a stop at current state
        const AccelPlan stop =
            planDecelToStop(initial_vel, initial_accel, max_jerk, max_decel);
        if (stop.phases.empty())
        {
            addTrajectoryPart({0.0, initial_pos, initial_vel, initial_accel, 0.0});
            return;
        }
        double t = 0.0, p = 0.0, v = initial_vel, a = initial_accel;
        applyPlan(stop, initial_pos, p, v, a, t);
        return;
    }

    // If we can stop before or at the destination, plan directly
    // Otherwise, decelerate to a full stop first, then plan from there
    const double stop_pos = closestPositionToStop(initial_pos, initial_vel, initial_accel,
                                                  max_jerk, max_decel);
    if (isInRangeInclusive(stop_pos, initial_pos, final_pos))
    {
        generateDirect(initial_pos, final_pos, initial_vel, initial_accel, max_vel,
                       max_accel, max_decel, max_jerk, 0.0);
    }
    else
    {
        const AccelPlan stop_plan =
            planDecelToStop(initial_vel, initial_accel, max_jerk, max_decel);
        double t = 0.0, p = 0.0, v = initial_vel, a = initial_accel;
        applyPlan(stop_plan, initial_pos, p, v, a, t);

        // Recursively plan from the stopped position to the final target
        generateDirect(initial_pos + p, final_pos, 0.0, 0.0, max_vel, max_accel,
                       max_decel, max_jerk, t);
    }
}

double JerkLimitedTrajectory1D::getPosition(const double t_sec) const
{
    const auto [part, dt] = getTrajPartAndDeltaTime(t_sec);
    return part.position + part.velocity * dt + 0.5 * part.acceleration * dt * dt +
           (part.jerk / 6.0) * dt * dt * dt;
}

double JerkLimitedTrajectory1D::getVelocity(const double t_sec) const
{
    const auto [part, dt] = getTrajPartAndDeltaTime(t_sec);
    return part.velocity + part.acceleration * dt + 0.5 * part.jerk * dt * dt;
}

double JerkLimitedTrajectory1D::getAcceleration(const double t_sec) const
{
    const auto [part, dt] = getTrajPartAndDeltaTime(t_sec);
    return part.acceleration + part.jerk * dt;
}

double JerkLimitedTrajectory1D::getTotalTime() const
{
    return trajectory_parts.back().end_time_sec;
}

std::pair<double, double> JerkLimitedTrajectory1D::getMinMaxPositions() const
{
    double min_p = trajectory_parts[0].position;
    double max_p = min_p;
    for (const auto& part : trajectory_parts)
    {
        min_p = std::min(min_p, part.position);
        max_p = std::max(max_p, part.position);
    }
    const double final_p = getPosition(getTotalTime());
    min_p                = std::min(min_p, final_p);
    max_p                = std::max(max_p, final_p);
    return {min_p, max_p};
}

const JerkLimitedTrajectory1D::TrajectoryPart& JerkLimitedTrajectory1D::getTrajectoryPart(
    const size_t index) const
{
    return trajectory_parts.at(index);
}

size_t JerkLimitedTrajectory1D::getNumTrajectoryParts() const
{
    return trajectory_parts.size();
}

size_t JerkLimitedTrajectory1D::getTrajectoryIndexAtTime(const double t_sec) const
{
    for (size_t i = 0; i < trajectory_parts.size(); ++i)
    {
        if (t_sec <= trajectory_parts[i].end_time_sec)
        {
            return i;
        }
    }
    return trajectory_parts.size() - 1;
}

std::pair<JerkLimitedTrajectory1D::TrajectoryPart, double>
JerkLimitedTrajectory1D::getTrajPartAndDeltaTime(double t_sec) const
{
    t_sec               = std::clamp(t_sec, 0.0, getTotalTime());
    const size_t index  = getTrajectoryIndexAtTime(t_sec);
    TrajectoryPart part = trajectory_parts[index];
    double prev_time;
    if (index == 0)
    {
        prev_time = 0.0;
    }
    else
    {
        prev_time = trajectory_parts[index - 1].end_time_sec;
    }
    double dt = t_sec - prev_time;
    return {part, dt};
}

void JerkLimitedTrajectory1D::addTrajectoryPart(const TrajectoryPart& part)
{
    trajectory_parts.push_back(part);
}

void JerkLimitedTrajectory1D::integrateState(double& p, double& v, double& a,
                                             const double dt, const double jerk)
{
    p += v * dt + 0.5 * a * dt * dt + (jerk / 6.0) * dt * dt * dt;
    v += a * dt + 0.5 * jerk * dt * dt;
    a += jerk * dt;
}

void JerkLimitedTrajectory1D::applyPlan(const AccelPlan& plan, const double start_pos,
                                        double& p, double& v, double& a, double& t)
{
    for (const auto& [jerk, duration] : plan.phases)
    {
        addTrajectoryPart({t + duration, start_pos + p, v, a, jerk});
        integrateState(p, v, a, duration, jerk);
        t += duration;
    }
}

JerkLimitedTrajectory1D::AccelPlan JerkLimitedTrajectory1D::planAccelProfile(
    const double initial_vel, const double initial_accel, const double final_vel,
    const double max_jerk, const double max_accel)
{
    AccelPlan result;

    // Determine the direction of travel so we can work with non-negative
    // speeds internally, then sign the output phases
    double dir;
    if (std::abs(initial_vel) < EPSILON)
    {
        // Starting from rest, direction is determined by the target velocity
        if (std::abs(final_vel) < EPSILON)
        {
            return result;
        }
        dir = std::copysign(1.0, final_vel);
    }
    else
    {
        dir = std::copysign(1.0, initial_vel);
    }

    const double initial_speed = std::abs(initial_vel);
    const double final_speed   = std::abs(final_vel);

    if (final_speed <= initial_speed)
    {
        // Not accelerating (speed is not increasing)
        return result;
    }

    const double mirror_a      = initial_accel * dir;
    const double delta_speed   = final_speed - initial_speed;
    const double t_to_max      = std::max(max_accel - mirror_a, 0.0) / max_jerk;
    const double t_max_to_zero = max_accel / max_jerk;
    const double delta_speed_min =
        (mirror_a + max_accel) / 2.0 * t_to_max + max_accel / 2.0 * t_max_to_zero;

    double t1, t2, t3;
    if (delta_speed >= delta_speed_min)
    {
        // Trapezoidal acceleration (reaches max_accel)
        t1 = t_to_max;
        t2 = (delta_speed - delta_speed_min) / max_accel;
        t3 = t_max_to_zero;
    }
    else
    {
        // Triangular acceleration (does not reach max_accel)
        const double peak_accel = std::min(
            std::sqrt(max_jerk * delta_speed + mirror_a * mirror_a / 2.0), max_accel);

        t1 = (peak_accel - mirror_a) / max_jerk;
        t2 = 0.0;
        t3 = peak_accel / max_jerk;
    }

    double p = 0.0, v = initial_speed, a = mirror_a;

    // Phase 1: increase acceleration to max_accel at +max_jerk
    if (t1 > 0)
    {
        result.phases.push_back({max_jerk * dir, t1});
        integrateState(p, v, a, t1, max_jerk);
    }

    // Phase 2: hold constant acceleration (zero jerk)
    if (t2 > 0)
    {
        result.phases.push_back({0.0, t2});
        integrateState(p, v, a, t2, 0.0);
    }

    // Phase 3: decrease acceleration to zero at -max_jerk
    if (t3 > 0)
    {
        result.phases.push_back({-max_jerk * dir, t3});
        integrateState(p, v, a, t3, -max_jerk);
    }

    result.total_distance = p * dir;
    return result;
}

JerkLimitedTrajectory1D::AccelPlan JerkLimitedTrajectory1D::planDecelToStop(
    const double initial_vel, const double initial_accel, const double max_jerk,
    const double max_decel)
{
    AccelPlan result;

    if (std::abs(initial_vel) < EPSILON && std::abs(initial_accel) < EPSILON)
    {
        // Already stopped, nothing to do
        return result;
    }

    // Work in a positive-velocity frame internally, then sign the output so
    // that phases and total_distance are in real-world coordinates
    double dir;
    if (std::abs(initial_vel) < EPSILON)
    {
        dir = 1.0;
    }
    else
    {
        dir = std::copysign(1.0, initial_vel);
    }
    const double abs_v = std::abs(initial_vel);
    double a           = initial_accel * dir;

    double p = 0.0, v = abs_v;

    // If acceleration exceeds the deceleration limit, bring it up to
    // -max_decel first
    if (a < -max_decel)
    {
        const double t = (-max_decel - a) / max_jerk;
        result.phases.push_back({max_jerk * dir, t});
        integrateState(p, v, a, t, max_jerk);
    }

    // Velocity is already zero, just cancel residual acceleration
    if (std::abs(v) < EPSILON)
    {
        if (a > 0.0)
        {
            const double t = a / max_jerk;
            result.phases.push_back({-max_jerk * dir, t});
            integrateState(p, v, a, t, -max_jerk);
        }
        else if (a < 0.0)
        {
            const double t = -a / max_jerk;
            result.phases.push_back({max_jerk * dir, t});
            integrateState(p, v, a, t, max_jerk);
        }
        result.total_distance = p * dir;
        return result;
    }

    const double delta_vel        = -v;
    const double t_to_max_decel   = std::max(a + max_decel, 0.0) / max_jerk;
    const double t_max_decel_zero = max_decel / max_jerk;
    const double delta_vel_min =
        (a - max_decel) / 2.0 * t_to_max_decel - max_decel / 2.0 * t_max_decel_zero;

    double t1, t2, t3;
    if (delta_vel <= delta_vel_min)
    {
        // Trapezoidal deceleration (reaches -max_decel)
        t1 = t_to_max_decel;
        t2 = (delta_vel_min - delta_vel) / max_decel;
        t3 = t_max_decel_zero;
    }
    else
    {
        // Triangular deceleration (does not reach -max_decel)
        const double peak_decel = std::max(
            -std::sqrt(std::max(0.0, (a * a - 2.0 * max_jerk * delta_vel) / 2.0)),
            -max_decel);

        t1 = (a - peak_decel) / max_jerk;
        t2 = 0.0;
        t3 = -peak_decel / max_jerk;
    }

    // Phase 1: decrease acceleration to -max_decel at -max_jerk
    if (t1 > 0)
    {
        result.phases.push_back({-max_jerk * dir, t1});
        integrateState(p, v, a, t1, -max_jerk);
    }

    // Phase 2: hold constant deceleration (zero jerk)
    if (t2 > 0)
    {
        result.phases.push_back({0.0, t2});
        integrateState(p, v, a, t2, 0.0);
    }

    // Phase 3: increase acceleration to zero at +max_jerk
    if (t3 > 0)
    {
        result.phases.push_back({max_jerk * dir, t3});
        integrateState(p, v, a, t3, max_jerk);
    }

    result.total_distance = p * dir;
    return result;
}

double JerkLimitedTrajectory1D::closestPositionToStop(const double initial_pos,
                                                      const double initial_vel,
                                                      const double initial_accel,
                                                      const double max_jerk,
                                                      const double max_decel)
{
    const auto [total_distance, phases] =
        planDecelToStop(initial_vel, initial_accel, max_jerk, max_decel);
    return initial_pos + total_distance;
}

void JerkLimitedTrajectory1D::generateDirect(
    const double start_pos, const double final_pos, const double initial_vel,
    const double initial_accel, const double max_vel, const double max_accel,
    const double max_decel, const double max_jerk, const double time_offset)
{
    const double displacement = final_pos - start_pos;
    const double direction    = std::copysign(1, displacement);
    const double distance     = std::abs(displacement);

    if (distance < EPSILON)
    {
        return;
    }

    auto totalDistNoCruise = [&](const double peak_speed)
    {
        const double signed_peak = direction * peak_speed;
        const double dist_accel  = std::abs(
             planAccelProfile(initial_vel, initial_accel, signed_peak, max_jerk, max_accel)
                 .total_distance);
        const double dist_decel = std::abs(
            planDecelToStop(signed_peak, 0.0, max_jerk, max_decel).total_distance);
        return dist_accel + dist_decel;
    };

    double peak_speed;
    double time_cruise;

    const double dist_max_vel = totalDistNoCruise(max_vel);
    if (dist_max_vel <= distance)
    {
        // Can reach max_vel, include a cruise phase
        peak_speed  = max_vel;
        time_cruise = (distance - dist_max_vel) / max_vel;
    }
    else
    {
        // Binary search for the peak speed that hits the desired distance
        // without a cruise phase
        double low = 0.0, high = max_vel;
        for (int i = 0; i < 30; ++i)
        {
            const double mid = (low + high) * 0.5;
            if (totalDistNoCruise(mid) < distance)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }
        peak_speed  = (low + high) * 0.5;
        time_cruise = 0.0;
    }

    const double signed_peak = direction * peak_speed;

    double p = 0.0;
    double v = initial_vel;
    double a = initial_accel;
    double t = time_offset;

    // Acceleration phases
    if (peak_speed > std::abs(v))
    {
        applyPlan(planAccelProfile(v, a, signed_peak, max_jerk, max_accel), start_pos, p,
                  v, a, t);
    }

    // Cruise phase (constant velocity)
    if (time_cruise > 0.0)
    {
        addTrajectoryPart({t + time_cruise, start_pos + p, v, a, 0.0});
        p += v * time_cruise;
        t += time_cruise;
    }

    // Deceleration phases
    if (peak_speed > 0.0)
    {
        v = signed_peak;
        a = 0.0;
        applyPlan(planDecelToStop(signed_peak, 0.0, max_jerk, max_decel), start_pos, p, v,
                  a, t);
    }
}
