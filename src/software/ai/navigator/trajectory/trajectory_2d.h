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
     * Checks if this 2D trajectory terminates at approximately the same destination
     * position as another trajectory.
     *
     * @param other The other trajectory to compare against.
     * @param threshold The maximum allowable distance in metres between the two
     * destinations.
     * @return True if the distance between destinations is less than or equal to the
     * threshold. False otherwise.
     */
    bool hasSameDestination(const Trajectory<Point, Vector, Vector>& other,
                            double threshold) const override
    {
        return distance(getDestination(), other.getDestination()) <= threshold;
    }
};
