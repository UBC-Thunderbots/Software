#pragma once

#include "software/geom/vector.h"

/**
 * A hybrid reciprocal velocity obstacle.
 */
class VelocityObstacle
{
   public:
    VelocityObstacle() = default;

    // The position of the apex of the hybrid reciprocal velocity obstacle.
    Vector apex_;

    // The direction of the first side of the hybrid reciprocal velocity obstacle.
    Vector side1_;

    // The direction of the second side of the hybrid reciprocal velocity obstacle.
    Vector side2_;
};
