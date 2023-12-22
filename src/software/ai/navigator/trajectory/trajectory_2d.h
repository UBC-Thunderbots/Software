#pragma once

#include "software/ai/navigator/trajectory/trajectory.hpp"
#include "software/geom/bounding_box.h"
#include "software/geom/point.h"

class Trajectory2D : virtual public Trajectory<Point, Vector, Vector>
{
   public:
    virtual ~Trajectory2D() = default;

    /**
     * Get a list of bounding boxes that this trajectory passes through
     * @return bounding boxes which this trajectory passes through
     */
    virtual std::vector<BoundingBox> getBoundingBoxes() const = 0;
};
