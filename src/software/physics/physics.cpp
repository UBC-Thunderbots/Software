#include "software/physics/physics.h"

#include <math.h>

Point calculateFuturePosition(const Point &initial_position,
                              const Vector &initial_velocity,
                              const Vector &constant_acceleration,
                              const Duration &duration_in_future)
{
    double seconds_in_future = duration_in_future.toSeconds();
    // Using second equation of motion from https://physics.info/motion-equations/
    // ∆s = v0*t + ½a*t^2
    double x1 = initial_position.x() + initial_velocity.x() * seconds_in_future +
                0.5 * constant_acceleration.x() * pow(seconds_in_future, 2);
    double y1 = initial_position.y() + initial_velocity.y() * seconds_in_future +
                0.5 * constant_acceleration.y() * pow(seconds_in_future, 2);

    return Point(x1, y1);
}

Vector calculateFutureVelocity(const Vector &initial_velocity,
                               const Vector &constant_acceleration,
                               const Duration &duration_in_future)
{
    double seconds_in_future = duration_in_future.toSeconds();
    double vx1 = initial_velocity.x() + seconds_in_future * constant_acceleration.x();
    double vy1 = initial_velocity.y() + seconds_in_future * constant_acceleration.y();
    return Vector(vx1, vy1);
}

EuclideanTo4Wheel::EuclideanTo4Wheel(float front_wheel_angle_deg, float back_wheel_angle_deg)
{
    // TODO: replace with Thunderloop polling rate constant
    delta_t_ = 1.0 / 200;

    // TODO: check wheel angles and orientations
    wheel_to_euclidean_velocity_coupling_matrix_ << -1/sin(front_wheel_angle_deg), -1/sin(back_wheel_angle_deg),
                                                    1/sin(back_wheel_angle_deg), 1/sin(front_wheel_angle_deg),
                                                    1/cos(front_wheel_angle_deg), -1/cos(back_wheel_angle_deg),
                                                    -1/cos(back_wheel_angle_deg), 1/cos(front_wheel_angle_deg),
                                                    1, 1, 1, 1;
}

WheelSpeeds EuclideanTo4Wheel::convert_to_wheel_speeds(EuclideanSpeeds euclidean_speeds)
{
    return WheelSpeeds();
}

EuclideanSpeeds EuclideanTo4Wheel::convert_to_euclidean_speeds(WheelSpeeds wheel_speeds)
{
    return EuclideanSpeeds();
}

float EuclideanTo4Wheel::acceleration_to_velocity(float acceleration, float current_velocity)
{
    return acceleration * delta_t_ + current_velocity;
}
