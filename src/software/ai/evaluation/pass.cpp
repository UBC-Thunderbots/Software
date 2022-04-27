#include "software/ai/evaluation/pass.h"


Duration getTimeToOrientationForRobot(const Angle& current_orientation,
                                      const Angle& desired_orientation,
                                      const double& max_velocity,
                                      const double& max_acceleration,
                                      const AngularVelocity& initial_angular_velocity)
{
    double dist = current_orientation.minDiff(desired_orientation).toRadians();
    double initial_ang_vel_rad_per_sec = initial_angular_velocity.toRadians();
    return getTimeToTravelDistance(dist, max_velocity, max_acceleration, initial_ang_vel_rad_per_sec, 0);
}

Duration getTimeToPositionForRobot(const Point& start, const Point& dest,
                                   const double max_velocity,
                                   const double max_acceleration,
                                   const double tolerance_meters,
                                   const Vector& initial_velocity,
                                   const Vector& final_velocity)
{
    Vector dist_vector = dest - start;
    double dist = std::max(0.0, dist_vector.length() - tolerance_meters);

    // To simplify the calculations we will solve this problem with 1D kinematics
    // by taking the component of the velocities projected onto the vector pointing
    // towards the destination
    double initial_velocity_1d = initial_velocity.dot(dist_vector.normalize());
    double final_velocity_1d = final_velocity.dot(dist_vector.normalize());

    return getTimeToTravelDistance(dist, max_velocity, max_acceleration, initial_velocity_1d, final_velocity_1d);
}

Duration getTimeToTravelDistance(const double distance,
                                 const double max_velocity,
                                 const double max_acceleration,
                                 const double initial_velocity,
                                 const double final_velocity)
{
    // Bound all values to be realistic
    double d_total = std::max(0.0, distance);
    double v_max = std::max(0.0, max_velocity);
    double v_i = std::min(initial_velocity, max_velocity);
    double v_f = std::clamp(final_velocity, 0.0, max_velocity);
    double a_max = std::max(1e-6, max_acceleration);

    // Minimum distance required to accelerate/decelerate from initial to final velocity
    double dist_required_to_reach_v_f = std::abs(std::pow(v_f, 2) - std::pow(v_i, 2)) / (2 * a_max);
    if (dist_required_to_reach_v_f > d_total)
    {
        // Accelerating/decelerating instantly from initial to final velocity is not possible with in the
        // given distance, given the robot's max acceleration, therefore the robot can not reach the
        // desired final velocity.
        // Calculate how long it will take for the robot to accelerate towards final velocity over distance
        double a_max_signed = a_max;
        if (v_f < v_i)
        {
            // Robot must decelerate, so it has negative acceleration
            a_max_signed *= -1.0;
        }
        // t = (-Vi + sqrt(Vi^2 + 2 * a * d)) / a
        double t_total = (-v_i + std::sqrt(std::pow(v_i, 2) + 2 * a_max_signed * d_total)) / a_max_signed;
        return Duration::fromSeconds(t_total);
    }

    // Total travel time if the robot is always accelerating at max acceleration (i.e. accelerating to
    // the highest value possible given the distance, and then decelerating to final velocity)
    double t_total = -(v_i + v_f - std::sqrt(2 * (2 * a_max * d_total + std::pow(v_i, 2) + std::pow(v_f, 2)))) / a_max;

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
        double d_cruising = d_total - d_accel - d_decel;
        double t_cruising = d_cruising / v_max;

        t_total = t_accel + t_cruising + t_decel;
    }

    return Duration::fromSeconds(t_total);
}