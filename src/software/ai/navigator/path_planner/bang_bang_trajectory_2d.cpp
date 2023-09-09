#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

BangBangTrajectory2D::BangBangTrajectory2D(const Point& initial_pos,
                                           const Point& final_pos,
                                           const Vector& initial_vel, double max_vel,
                                           double max_accel, double max_decel)
{
    generate(initial_pos, final_pos, initial_vel, max_vel, max_accel, max_decel);
}

void BangBangTrajectory2D::generate(const Point& initial_pos, const Point& final_pos,
                                    const Vector& initial_vel, double max_vel,
                                    double max_accel, double max_decel)
{
    // The following implementation is based on section IV of the paper
    // "Trajectory generation for four wheeled omnidirectional vehicles"

    // Take the x and y components of the max velocity, acceleration, and deceleration
    // based on the angle alpha so the overall 2D trajectory abides by the kinematic
    // constraints.

    // Alpha starts at PI/4 (45 deg) which will have the x and y components equivalent to
    // each other. We then increment alpha by PI/(8*2^i) where i is the iteration of
    // binary search we're at. This will result in alpha being in the range of 0 rad (no
    // y-component) to PI/2 rad (no x-component).
    double alpha     = M_PI / 4.0;
    double increment = M_PI / 8.0;

    // Use binary search to find the ideal alpha which results in the x and y trajectories
    // to take basically the same amount of time to reach the destination
    while (increment > 1e-7)
    {
        const double cos = std::cos(alpha);
        const double sin = std::sin(alpha);

        x_trajectory.generate(initial_pos.x(), final_pos.x(), initial_vel.x(),
                              max_vel * cos, max_accel * cos, max_decel * cos);
        y_trajectory.generate(initial_pos.y(), final_pos.y(), initial_vel.y(),
                              max_vel * sin, max_accel * sin, max_decel * sin);

        const double x_time_sec = x_trajectory.getTotalTime();
        const double y_time_sec = y_trajectory.getTotalTime();
        if (std::abs(x_time_sec - y_time_sec) < TRAJ_ACCURACY_TOLERANCE_SEC)
        {
            break;
        }

        if (x_time_sec > y_time_sec)
        {
            // x trajectory takes longer, so decrease the alpha resulting in it getting
            // a larger component of the kinematic constraints relative to y and reaching
            // the destination faster
            alpha -= increment;
        }
        else
        {
            alpha += increment;
        }

        increment /= 2.0;
    }
}

Point BangBangTrajectory2D::getPosition(double t_sec) const
{
    return Point(x_trajectory.getPosition(t_sec), y_trajectory.getPosition(t_sec));
}

Vector BangBangTrajectory2D::getVelocity(double t_sec) const
{
    return Vector(x_trajectory.getVelocity(t_sec), y_trajectory.getVelocity(t_sec));
}

Vector BangBangTrajectory2D::getAcceleration(double t_sec) const
{
    return Vector(x_trajectory.getAcceleration(t_sec), y_trajectory.getAcceleration(t_sec));
}

double BangBangTrajectory2D::getTotalTime() const
{
    return std::max(x_trajectory.getTotalTime(), y_trajectory.getTotalTime());
}

Rectangle BangBangTrajectory2D::getBoundingBox() const
{
    std::pair<double, double> x_min_max = x_trajectory.getMinMaxPositions();
    std::pair<double, double> y_min_max = y_trajectory.getMinMaxPositions();
    // If min max are the same, shift them slightly so a valid rectangle can
    // be created
    if (std::abs(x_min_max.first - x_min_max.second) <= FIXED_EPSILON)
    {
        x_min_max.first -= 0.01;
        x_min_max.second += 0.01;
    }
    if (std::abs(y_min_max.first - y_min_max.second) <= FIXED_EPSILON)
    {
        y_min_max.first -= 0.01;
        y_min_max.second += 0.01;
    }
    return Rectangle({x_min_max.first, y_min_max.first},
                     {x_min_max.second, y_min_max.second});
}
