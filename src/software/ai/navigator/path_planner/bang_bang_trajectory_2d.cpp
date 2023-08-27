#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

void BangBangTrajectory2D::generate(const Point& initial_pos, const Point& final_pos,
                                    const Vector& initial_vel, double max_vel,
                                    double max_accel, double max_decel)
{
    // TODO: Consider moving the constants to proto parameters. Why are these values used?
    double increment = M_PI / 8.0;
    double alpha     = M_PI / 4.0;

    // Binary search tuning the max constants for x and y components
    // until their total time is basically equal
    while (increment > 1e-7)
    {
        const double cos = std::cos(alpha);
        const double sin = std::sin(alpha);

        x_trajectory.generate(initial_pos.x(), final_pos.x(), initial_vel.x(),
                              max_vel * cos, max_accel * cos, max_decel * cos);
        y_trajectory.generate(initial_pos.y(), final_pos.y(), initial_vel.y(),
                              max_vel * sin, max_accel * sin, max_decel * sin);

        const Duration x_time = x_trajectory.getTotalTime();
        const Duration y_time = y_trajectory.getTotalTime();
        if (std::abs((x_time - y_time).toSeconds()) < ACCURACY)
        {
            break;
        }

        if (x_time > y_time)
        {
            alpha -= increment;
        }
        else
        {
            alpha += increment;
        }

        increment /= 2.0;
    }
}

Point BangBangTrajectory2D::getPosition(Duration t) const
{
    return Point(x_trajectory.getPosition(t), y_trajectory.getPosition(t));
}

Vector BangBangTrajectory2D::getVelocity(Duration t) const
{
    return Vector(x_trajectory.getVelocity(t), x_trajectory.getVelocity(t));
}

Vector BangBangTrajectory2D::getAcceleration(Duration t) const
{
    return Vector(x_trajectory.getAcceleration(t), x_trajectory.getAcceleration(t));
}

Duration BangBangTrajectory2D::getTotalTime() const
{
    return std::max(x_trajectory.getTotalTime(), y_trajectory.getTotalTime());
}
