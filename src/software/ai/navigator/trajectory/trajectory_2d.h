#pragma once

#include "software/ai/navigator/trajectory/trajectory.hpp"
#include "software/geom/algorithms/distance.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

class Trajectory2D : virtual public Trajectory<Point, Vector, Vector>
{
   public:
    virtual ~Trajectory2D() = default;

    /**
     * Get a list of bounding boxes that this trajectory passes through
     * @return bounding boxes which this trajectory passes through
     */
    virtual std::vector<Rectangle> getBoundingBoxes() const = 0;

    /**
     * Check if this trajectory is meaningfully different from another trajectory.
     * @param other The other trajectory to compare to
     * @param threshold The threshold above which the trajectories are considered
     * different
     * @return True if the trajectories are different, false otherwise
     */
    bool isNew(const Trajectory<Point, Vector, Vector>& other,
               double threshold) const override
    {
        return distance(getDestination(), other.getDestination()) > threshold;
    }
};
