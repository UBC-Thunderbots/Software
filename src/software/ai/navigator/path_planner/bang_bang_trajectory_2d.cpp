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
    /**
     * The following implementation is based on section IV of the paper
     * "Trajectory generation for four wheeled omnidirectional vehicles"
     *
     * A 2D trajectories is represented as 2x 1D trajectories (one for x and one for y)
     * which take the same amount of time to reach the destination.
     *
     * The max velocity, acceleration, and deceleration are assumed to be the kinematic
     * constraints in 2D space, as opposed to the constraints for individual x and y
     * components. To find the components of these constraints for each axis which will
     * give us two trajectories with (almost) equal durations, we will use binary search
     * over the components of the given kinematic constraints.
     *
     * We will start with alpha = PI/4 (45 deg) which will have the x and y components
     * equivalent to each other. Depending on which 1D trajectory takes longer to reach
     * the destination, we will either increase or decrease alpha by increment
     * (= PI/(8*2^i) where i is the iteration of binary search we're at). This will
     * result in the component which is taking longer to reach the destination to get a
     * larger component of the kinematic constraints relative to the other component which
     * reaches the destination faster.
     */

    double alpha     = M_PI / 4.0;
    double increment = M_PI / 8.0;

    while (increment > 1e-3)
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
            // The duration of the trajectories are close enough, so we can stop the
            // binary search.
            break;
        }

        if (x_time_sec > y_time_sec)
        {
            // x trajectory takes longer, so decrease the alpha resulting in the x
            // trajectory getting a larger component of the kinematic constraints relative
            // to y and reaching the destination faster
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
    return Vector(x_trajectory.getAcceleration(t_sec),
                  y_trajectory.getAcceleration(t_sec));
}

double BangBangTrajectory2D::getTotalTime() const
{
    return std::max(x_trajectory.getTotalTime(), y_trajectory.getTotalTime());
}

std::vector<BoundingBox> BangBangTrajectory2D::getBoundingBoxes() const
{
    std::pair<double, double> x_min_max = x_trajectory.getMinMaxPositions();
    std::pair<double, double> y_min_max = y_trajectory.getMinMaxPositions();
    // If min max are the same, shift them slightly so a valid bounding box can
    // be created
    if (std::abs(x_min_max.first - x_min_max.second) <=
        std::numeric_limits<double>::epsilon())
    {
        x_min_max.first -= 0.001;
        x_min_max.second += 0.001;
    }
    if (std::abs(y_min_max.first - y_min_max.second) <=
        std::numeric_limits<double>::epsilon())
    {
        y_min_max.first -= 0.001;
        y_min_max.second += 0.001;
    }
    return {BoundingBox({x_min_max.first, y_min_max.first},
                        {x_min_max.second, y_min_max.second})};
}
