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
     * Check if this trajectory is meaningfully equal to another trajectory.
     * @param other The other trajectory to compare to
     * @param threshold The threshold below which the trajectories are considered
     * equal
     * @return True if the trajectories are equal, false otherwise
     */
    bool equals(const Trajectory<Point, Vector, Vector>& other,
                double threshold) const override
    {
        return distance(getDestination(), other.getDestination()) <= threshold;
    }
};
