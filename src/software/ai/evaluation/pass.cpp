#include "software/ai/evaluation/pass.h"

Duration getTimeToOrientationForRobot(const Robot& robot,
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

    double dist = robot.orientation().minDiff(desired_orientation).toRadians();

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

Duration getTimeToPositionForRobot(const Robot& robot, const Point& dest,
                                   const double& max_velocity,
                                   const double& max_acceleration,
                                   const double& tolerance_meters)
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

    double dist = std::max(0.0, (robot.position() - dest).length() - tolerance_meters);

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
