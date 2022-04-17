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

		// The direction of the right side of the hybrid reciprocal velocity obstacle.
		Vector right_side;

		// The direction of the left side of the hybrid reciprocal velocity obstacle.
		Vector left_side;
};
