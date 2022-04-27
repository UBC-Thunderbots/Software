#include "software/ai/evaluation/pass.h"

Duration getTimeToOrientationForRobot(const Angle& current_orientation,
                                      const Angle& desired_orientation,
                                      const double& max_velocity,
                                      const double& max_acceleration)
{
    // We assume a linear acceleration profile:
    // (1) velocity = MAX_ACCELERATION*time
    // we integrate (1) to get:
    // (2) displacement = MAX_ACCELERATION/2 * time^2
    // we rearrange to get:
    // (3) time = sqrt(2 * displacement / MAX_ACCELERATION)
    // we sub. (3) into (1) to get:
    // (4) velocity = MAX_ACCELERATION*sqrt(2 * displacement / MAX_ACCELERATION)
    // and rearrange to get:
    // (5) displacement = (velocity / MAX_ACCELERATION)^2 * MAX_ACCELERATION/2
    // We re-arrange (3) to get:
    // (6) displacement = time^2 * MAX_ACCELERATION/2

    double dist = current_orientation.minDiff(desired_orientation).toRadians();

    // Calculate the distance required to reach max possible velocity of the robot
    // using (5)
    double dist_to_max_possible_vel =
            std::pow(max_velocity / max_acceleration, 2) * max_acceleration / 2;

    // Calculate how long we'll accelerate for using (3), taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    double acceleration_time =
            std::sqrt(2 * std::min(dist / 2, dist_to_max_possible_vel) / max_acceleration);

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
            std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the target angle is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}

Duration getTimeToPositionForRobot(const Point& start, const Point& dest,
                                   const double max_velocity,
                                   const double max_acceleration,
                                   const double tolerance_meters)
{
    // We assume a linear acceleration profile:
    // (1) velocity = MAX_ACCELERATION*time
    // we integrate (1) to get:
    // (2) displacement = MAX_ACCELERATION/2 * time^2
    // we rearrange to get:
    // (3) time = sqrt(2 * displacement / MAX_ACCELERATION)
    // we sub. (3) into (1) to get:
    // (4) velocity = MAX_ACCELERATION*sqrt(2 * displacement / MAX_ACCELERATION)
    // and rearrange to get:
    // (5) displacement = (velocity / MAX_ACCELERATION)^2 * MAX_ACCELERATION/2
    // We re-arrange (3) to get:
    // (6) displacement = time^2 * MAX_ACCELERATION/2

    double dist = std::max(0.0, (start - dest).length() - tolerance_meters);

    // Calculate the distance required to reach max possible velocity of the robot
    // using (5)
    double dist_to_max_possible_vel =
            std::pow(max_velocity / max_acceleration, 2) * max_acceleration / 2;

    // Calculate how long we'll accelerate for using (3), taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    double acceleration_time =
            std::sqrt(2 * std::min(dist / 2, dist_to_max_possible_vel) / max_acceleration);

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
            std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the receiver point is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}

Duration getTimeToPositionForRobot(const Point& start, const Point& dest,
                                   const double max_velocity,
                                   const double max_acceleration,
                                   const Vector& initial_velocity = Vector(),
                                   const Vector& final_velocity = Vector())
{
    Vector dist_vector = start - dest;
    double dist = dist_vector.length();

    // To simplify the calculations we will solve this problem with 1D kinematics
    // by taking the component of the velocities pointing towards the destination
    double initial_velocity_1d = initial_velocity.dot(dist_vector);
    // TODO: Not taking into account the final velocity...
    double final_velocity_1d = final_velocity.dot(dist_vector);

    // Calculate the distance required to reach max possible velocity of the robot
    // d = (Vf^2 - Vi^2) / (2 * a)
    double dist_to_max_possible_vel =
            (std::pow(max_velocity, 2) - std::pow(initial_velocity_1d, 2)) / (2 * max_acceleration);

    // Calculate how long we'll accelerate for, taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    // d = Vi * t + a * t^2 / 2, solving for t = (-Vi + sqrt(Vi^2 + 2ad)) / a
    double acceleration_time = (-initial_velocity_1d + std::sqrt(std::pow(initial_velocity_1d, 2) + 2 * max_acceleration * std::min(dist / 2, dist_to_max_possible_vel))) / max_acceleration;

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
            std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the receiver point is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}

Duration getTimeToTravelDistance(const double distance,
                                 const double v_max,
                                 const double a_max,
                                 const double v_i = 0,
                                 const double v_f = 0)
{
    // TODO: all values must be >= 0 (other than a_max which should be a > 0)
    // TODO: v_i <= v_max  AND  v_f <= v_max

    // Distance to accelerate/decelerate from initial to final velocity
    double decel_dist = std::abs(std::pow(v_f, 2) - std::pow(v_i, 2)) / (2 * a_max);
    if (decel_dist > distance)
    {
        // Accelerating/decelerating instantly from initial to final velocity is not possible with in the
        // given distance, given the object's max acceleration, therefore the object can not reach the
        // desired final velocity.
        // Calculate how long it will take for the object to accelerate towards final velocity over distance
        double a_max_signed = a_max;
        if (v_f < v_i)
        {
            // Robot must decelerate, so it has negative acceleration
            a_max_signed *= -1.0;
        }
        double t_total = (-v_i + std::sqrt(2 * a_max_signed * distance + std::pow(v_i, 2))) / a_max_signed;
        return Duration::fromSeconds(t_total);
    }

    // Total travel time if the object is always accelerating at max acceleration (i.e. accelerating to
    // the highest value possible given the distance, and then decelerating to final velocity)
    double t_total = -(v_i + v_f - std::sqrt(2 * (2 * a_max * distance + std::pow(v_i, 2) + std::pow(v_f, 2)))) / a_max;

    // The max velocity reached if moving given the above condition
    double v_max_reached = (a_max * t_total + v_f + v_i) / 2;

    if(v_max_reached > v_max)
    {
        // If the robot is always accelerating, it will be going faster than max velocity, so instead
        // we will divide the problem into 3 sections:
        // The robot will be (1) accelerating, (2) cruising at max velocity, then (3) decelerating.
        // Calculate travel time during (1) and (3):
        double t_accel = (v_max - v_i) / a_max;
        double t_decel = (v_f - v_max) / -a_max;

        // To calculate (2) we will need to know the distance travelled while cruising
        double d_accel = t_accel * (v_i + v_max) / 2;
        double d_decel = t_decel * (v_f + v_max) / 2;
        double d_cruising = distance - d_accel - d_decel;
        double t_cruising = d_cruising / v_max;

        t_total = t_accel + t_cruising + t_decel;
    }

    return Duration::fromSeconds(t_total);
}

Duration getTimeToOrientationForRobot(const Angle& current_orientation,
                                      const Angle& desired_orientation,
                                      const double& max_velocity,
                                      const double& max_acceleration,
                                      const AngularVelocity& initial_angular_velocity = AngularVelocity::zero())
{
    // TODO: Not taking into account the final velocity...
    double dist = current_orientation.minDiff(desired_orientation).toRadians();
    double initial_ang_vel_rad_per_sec = initial_angular_velocity.toRadians();

    // Calculate the distance required to reach max possible velocity of the robot
    // d = (Vf^2 - Vi^2) / (2 * a)
    double dist_to_max_possible_vel =
            (std::pow(max_velocity, 2) - std::pow(initial_ang_vel_rad_per_sec, 2)) / (2 * max_acceleration);

    // Calculate how long we'll accelerate for, taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    // d = Vi * t + a * t^2 / 2, solving for t = (-Vi + sqrt(Vi^2 + 2ad)) / a
    double acceleration_time = (-initial_ang_vel_rad_per_sec + std::sqrt(std::pow(initial_ang_vel_rad_per_sec, 2) + 2 * max_acceleration * std::min(dist / 2, dist_to_max_possible_vel))) / max_acceleration;

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
            std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the target angle is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}