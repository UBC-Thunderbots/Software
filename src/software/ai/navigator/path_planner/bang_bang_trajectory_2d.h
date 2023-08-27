#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"
#include "software/geom/point.h"

class BangBangTrajectory2D : public ITrajectory<Point, Vector, Vector>
{
   public:
    BangBangTrajectory2D() = default;

    void generate(const Point& initial_pos, const Point& final_pos,
                  const Vector& initial_vel, double max_vel, double max_accel,
                  double max_decel);

    Point getPosition(Duration t) const override;

    Vector getVelocity(Duration t) const override;

    Vector getAcceleration(Duration t) const override;

    Duration getTotalTime() const override;


   private:
    BangBangTrajectory1D x_trajectory;
    BangBangTrajectory1D y_trajectory;
    const double ACCURACY = 1e-2;
};
