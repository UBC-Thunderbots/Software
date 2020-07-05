#include "software/new_geom/util/physics.h"

Point calculateFuturePosition(const Point &initial_position,
                              const Vector &intial_velocity, const Vector &acceleration,
                              double time_in_future)
{
    // Using second equation of motion from https://physics.info/motion-equations/
    // ∆s = v0t + ½at2
    double x1 = intial_velocity.x() * time_in_future +
                0.5 * acceleration.x() * pow(time_in_future, 2);
    double y1 = intial_velocity.y() * time_in_future +
                0.5 * acceleration.y() * pow(time_in_future, 2);

    return Point(x1, y1);
}

Vector calculateFutureVelocity(const Vector &intial_velocity, const Vector &acceleration,
                               double time_in_future)
{
    double vx1 = intial_velocity.x() + time_in_future * acceleration.x();
    double vy1 = intial_velocity.y() + time_in_future * acceleration.y();
    return Vector(vx1, vy1);
}
