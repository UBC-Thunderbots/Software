#pragma once

#include <memory>

#include "software/ai/navigator/trajectory/jerk_limited_trajectory_1d.h"
#include "software/ai/navigator/trajectory/kinematic_constraints.h"
#include "software/ai/navigator/trajectory/trajectory_2d.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/vector.h"

class JerkLimitedTrajectory2D : public Trajectory2D
{
   public:
    JerkLimitedTrajectory2D() = default;

    JerkLimitedTrajectory2D(const Point& initial_pos, const Point& final_pos,
                            const Vector& initial_vel, const Vector& initial_accel,
                            const KinematicConstraints& constraints);

    void generate(const Point& initial_pos, const Point& final_pos,
                  const Vector& initial_vel, const Vector& initial_accel, double max_vel,
                  double max_accel, double max_decel, double max_jerk);

    Point getPosition(double t_sec) const override;
    Vector getVelocity(double t_sec) const override;
    Vector getAcceleration(double t_sec) const override;
    double getTotalTime() const override;
    std::vector<Rectangle> getBoundingBoxes() const override;

    static std::shared_ptr<Trajectory2D> generator(
        const Point& initial_pos, const Point& final_pos, const Vector& initial_vel,
        const Vector& initial_accel, const KinematicConstraints& constraints);

   private:
    JerkLimitedTrajectory1D x_trajectory;
    JerkLimitedTrajectory1D y_trajectory;
    static constexpr double TRAJ_ACCURACY_TOLERANCE_SEC = 0.01;
};
