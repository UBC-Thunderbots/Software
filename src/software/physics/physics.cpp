#include "software/physics/physics.h"

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
