#pragma once

#include "software/ai/navigator/obstacle/geom_obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"

template <typename GEOM_TYPE>
class TrajectoryObstacle : public GeomObstacle<GEOM_TYPE>
{
   public:
    TrajectoryObstacle() = delete;

    /**
     * Construct a TrajectoryObstacle with GEOM_TYPE
     *
     * @param geom GEOM_TYPE to make obstacle with
     * @param traj Trajectory which the obstacle is following
     */
    explicit TrajectoryObstacle(const GEOM_TYPE& geom, TrajectoryPath traj);

    bool contains(const Point& p, const double t_sec = 0) const override;
    double distance(const Point& p, const double t_sec = 0) const override;
    bool intersects(const Segment& segment, const double t_sec = 0) const override;

   private:
    const TrajectoryPath traj_;
};


template <typename GEOM_TYPE>
TrajectoryObstacle<GEOM_TYPE>::TrajectoryObstacle(const GEOM_TYPE& geom, TrajectoryPath traj) : GeomObstacle<GEOM_TYPE>(geom), traj_(std::move(traj))
{
}

template <typename GEOM_TYPE>
bool TrajectoryObstacle<GEOM_TYPE>::contains(const Point& p, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::contains(this->geom_, p);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        const Vector displacement = traj_.getPosition(t_sec) - traj_.getPosition(0);
        return ::contains(this->geom_, p - displacement);
    }
}

template <typename GEOM_TYPE>
double TrajectoryObstacle<GEOM_TYPE>::distance(const Point& p, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::distance(this->geom_, p);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        const Vector displacement = traj_.getPosition(t_sec) - traj_.getPosition(0);
        return ::distance(this->geom_, p - displacement);
    }
}

template <typename GEOM_TYPE>
bool TrajectoryObstacle<GEOM_TYPE>::intersects(const Segment& segment, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::intersects(this->geom_, segment);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        // const Vector displacement = traj_.getPosition(t_sec) - traj_.getPosition(0);
        return ::intersects(this->geom_, segment/* - displacement*/); // TODO (nima): This is being added in Mikhael's PR
    }
}
