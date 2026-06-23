#include "software/ai/navigator/trajectory/jerk_limited_trajectory_2d.h"

#include <cmath>

JerkLimitedTrajectory2D::JerkLimitedTrajectory2D(const Point& initial_pos,
                                                 const Point& final_pos,
                                                 const Vector& initial_vel,
                                                 const Vector& initial_accel,
                                                 const KinematicConstraints& constraints)
{
    generate(initial_pos, final_pos, initial_vel, initial_accel,
             constraints.getMaxVelocity(), constraints.getMaxAcceleration(),
             constraints.getMaxDeceleration(), constraints.getMaxJerk());
}

void JerkLimitedTrajectory2D::generate(const Point& initial_pos, const Point& final_pos,
                                       const Vector& initial_vel,
                                       const Vector& initial_accel, const double max_vel,
                                       const double max_accel, const double max_decel,
                                       const double max_jerk)
{
    // Bisect angle alpha to split 2D limits into x/y components so both
    // 1D trajectories finish at nearly the same time
    double alpha     = M_PI / 4.0;
    double increment = M_PI / 8.0;

    while (increment > 1e-3)
    {
        const double cos_alpha = std::cos(alpha);
        const double sin_alpha = std::sin(alpha);

        x_trajectory.generate(initial_pos.x(), final_pos.x(), initial_vel.x(),
                              initial_accel.x(), max_vel * cos_alpha,
                              max_accel * cos_alpha, max_decel * cos_alpha,
                              max_jerk * cos_alpha);
        y_trajectory.generate(initial_pos.y(), final_pos.y(), initial_vel.y(),
                              initial_accel.y(), max_vel * sin_alpha,
                              max_accel * sin_alpha, max_decel * sin_alpha,
                              max_jerk * sin_alpha);

        if (std::abs(x_trajectory.getTotalTime() - y_trajectory.getTotalTime()) <
            TRAJ_ACCURACY_TOLERANCE_SEC)
        {
            break;
        }

        if (x_trajectory.getTotalTime() > y_trajectory.getTotalTime())
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

Point JerkLimitedTrajectory2D::getPosition(const double t_sec) const
{
    return Point(x_trajectory.getPosition(t_sec), y_trajectory.getPosition(t_sec));
}

Vector JerkLimitedTrajectory2D::getVelocity(const double t_sec) const
{
    return Vector(x_trajectory.getVelocity(t_sec), y_trajectory.getVelocity(t_sec));
}

Vector JerkLimitedTrajectory2D::getAcceleration(const double t_sec) const
{
    return Vector(x_trajectory.getAcceleration(t_sec),
                  y_trajectory.getAcceleration(t_sec));
}

double JerkLimitedTrajectory2D::getTotalTime() const
{
    return std::max(x_trajectory.getTotalTime(), y_trajectory.getTotalTime());
}

std::vector<Rectangle> JerkLimitedTrajectory2D::getBoundingBoxes() const
{
    auto [x_min, x_max] = x_trajectory.getMinMaxPositions();
    auto [y_min, y_max] = y_trajectory.getMinMaxPositions();

    // Ensure a minimum width/height so bounding boxes are never degenerate
    constexpr double MIN_BOX_SIDE = 0.001;
    if (std::abs(x_min - x_max) <= MIN_BOX_SIDE)
    {
        x_min -= MIN_BOX_SIDE;
        x_max += MIN_BOX_SIDE;
    }
    if (std::abs(y_min - y_max) <= MIN_BOX_SIDE)
    {
        y_min -= MIN_BOX_SIDE;
        y_max += MIN_BOX_SIDE;
    }

    return {Rectangle({x_min, y_min}, {x_max, y_max})};
}

std::shared_ptr<Trajectory2D> JerkLimitedTrajectory2D::generator(
    const Point& initial_pos, const Point& final_pos, const Vector& initial_vel,
    const Vector& initial_accel, const KinematicConstraints& constraints)
{
    return std::make_shared<JerkLimitedTrajectory2D>(initial_pos, final_pos, initial_vel,
                                                     initial_accel, constraints);
}
