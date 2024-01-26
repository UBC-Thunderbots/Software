#pragma once

#include "software/ai/navigator/obstacle/geom_obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_2d.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"

template <typename GEOM_TYPE>
class DynamicObstacle : public GeomObstacle<GEOM_TYPE>
{
   public:
    DynamicObstacle() = delete;

    /**
     * Construct a DynamicObstacle with GEOM_TYPE
     *
     * @param geom GEOM_TYPE to make obstacle with
     * @param traj Trajectory which the obstacle is following
     */
    explicit DynamicObstacle(const GEOM_TYPE& geom, const std::shared_ptr<const Trajectory2D> traj);

    bool contains(const Point& p, const double t_sec = 0) const override;
    double distance(const Point& p, const double t_sec = 0) const override;
    bool intersects(const Segment& segment, const double t_sec = 0) const override;

   private:
    const std::shared_ptr<const Trajectory2D> traj_;
};


template <typename GEOM_TYPE>
DynamicObstacle<GEOM_TYPE>::DynamicObstacle(const GEOM_TYPE& geom, const std::shared_ptr<const Trajectory2D> traj) : GeomObstacle<GEOM_TYPE>(geom), traj_(traj)
{
}

template <typename GEOM_TYPE>
bool DynamicObstacle<GEOM_TYPE>::contains(const Point& p, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::contains(this->geom_, p);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        const Vector displacement = traj_->getPosition(t_sec) - traj_->getPosition(0);
        return ::contains(this->geom_, p - displacement);
    }
}

template <typename GEOM_TYPE>
double DynamicObstacle<GEOM_TYPE>::distance(const Point& p, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::distance(this->geom_, p);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        const Vector displacement = traj_->getPosition(t_sec) - traj_->getPosition(0);
        return ::distance(this->geom_, p - displacement);
    }
}

template <typename GEOM_TYPE>
bool DynamicObstacle<GEOM_TYPE>::intersects(const Segment& segment, const double t_sec) const
{
    if (t_sec == 0)
    {
        return ::intersects(this->geom_, segment);
    }
    else
    {
        // Instead of shifting the obstacle, we will shift the point
        // in the opposite direction of the motion of obstacle
        // const Vector displacement = traj_->getPosition(t_sec) - traj_->getPosition(0);
        return ::intersects(this->geom_, segment/* - displacement*/); // TODO (nima): This is being added in Mikhael's PR
    }
}
