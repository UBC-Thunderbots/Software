#pragma once

#include "software/ai/navigator/obstacle/geom_obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"

template <typename GEOM_TYPE>
class ConstVelocityObstacle : public GeomObstacle<GEOM_TYPE>
{
   public:
    ConstVelocityObstacle() = delete;

    /**
     * Construct a ConstVelocityObstacle with GEOM_TYPE
     *
     * @param geom GEOM_TYPE to make obstacle with
     * @param velocity Velocity of the obstacle
     * @param max_time_horizon_sec Maximum time into the future the position of the
     * obstacle is predicted for in seconds. This is needed since the prediction of the
     * obstacle's position in the future becomes less accurate as time increases.
     */
    explicit ConstVelocityObstacle(const GEOM_TYPE& geom, Vector velocity,
                                   double max_time_horizon_sec);

    bool contains(const Point& p, const double t_sec = 0) const override;
    double distance(const Point& p, const double t_sec = 0) const override;
    double signedDistance(const Point& p, const double t_sec = 0) const override;
    bool intersects(const Segment& segment, const double t_sec = 0) const override;

   private:
    const Vector velocity_;
    const double max_time_horizon_sec_;
};


template <typename GEOM_TYPE>
ConstVelocityObstacle<GEOM_TYPE>::ConstVelocityObstacle(const GEOM_TYPE& geom,
                                                        Vector velocity,
                                                        double max_time_horizon_sec)
    : GeomObstacle<GEOM_TYPE>(geom),
      velocity_(std::move(velocity)),
      max_time_horizon_sec_(max_time_horizon_sec)
{
}

/**
 * Instead of shifting the obstacle, it is simpler to shift the point in the opposite
 * direction.
 */

template <typename GEOM_TYPE>
bool ConstVelocityObstacle<GEOM_TYPE>::contains(const Point& p, const double t_sec) const
{
    return ::contains(this->geom_,
                      p - velocity_ * std::min(t_sec, max_time_horizon_sec_));
}

template <typename GEOM_TYPE>
double ConstVelocityObstacle<GEOM_TYPE>::distance(const Point& p,
                                                  const double t_sec) const
{
    return ::distance(this->geom_,
                      p - velocity_ * std::min(t_sec, max_time_horizon_sec_));
}

template <typename GEOM_TYPE>
double ConstVelocityObstacle<GEOM_TYPE>::signedDistance(const Point& p,
                                                  const double t_sec) const
{
    return ::signedDistance(this->geom_,
                      p - velocity_ * std::min(t_sec, max_time_horizon_sec_));
}

template <typename GEOM_TYPE>
bool ConstVelocityObstacle<GEOM_TYPE>::intersects(const Segment& segment,
                                                  const double t_sec) const
{
    return ::intersects(this->geom_,
                        segment - velocity_ * std::min(t_sec, max_time_horizon_sec_));
}
