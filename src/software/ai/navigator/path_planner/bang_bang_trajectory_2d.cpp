#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

void BangBangTrajectory2D::generate(const Point& initial_pos, const Point& final_pos,
                                    const Vector& initial_vel, double max_vel,
                                    double max_accel, double max_decel)
{
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

        const Duration x_time = x_trajectory.getTotalTime();
        const Duration y_time = y_trajectory.getTotalTime();
        if (std::abs((x_time - y_time).toSeconds()) < TRAJ_ACCURACY_TOLERANCE_SEC)
        {
            break;
        }

        if (x_time > y_time)
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

    std::cout << "X vel" << std::endl;
    double time = 0;
    for (const auto& part : x_trajectory.getTrajectoryParts())
    {
        std::cout << Point(time, part.velocity) << ",";
        time = part.end_time.toSeconds();
    }
    std::cout << Point(time, 0.0) << ",";
    std::cout << std::endl;


    std::cout << "Y vel" << std::endl;
    time = 0;
    for (const auto& part : y_trajectory.getTrajectoryParts())
    {
        std::cout << Point(time, part.velocity) << ",";
        time = part.end_time.toSeconds();
    }
    std::cout << Point(time, 0.0) << ",";
    std::cout << std::endl;
}

Point BangBangTrajectory2D::getPosition(Duration t) const
{
    return Point(x_trajectory.getPosition(t), y_trajectory.getPosition(t));
}

Vector BangBangTrajectory2D::getVelocity(Duration t) const
{
    return Vector(x_trajectory.getVelocity(t), y_trajectory.getVelocity(t));
}

Vector BangBangTrajectory2D::getAcceleration(Duration t) const
{
    return Vector(x_trajectory.getAcceleration(t), y_trajectory.getAcceleration(t));
}

Duration BangBangTrajectory2D::getTotalTime() const
{
    return std::max(x_trajectory.getTotalTime(), y_trajectory.getTotalTime());
}
