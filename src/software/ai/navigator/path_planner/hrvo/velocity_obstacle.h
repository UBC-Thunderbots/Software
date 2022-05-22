#pragma once

#include "software/geom/vector.h"
#include "software/geom/circle.h"

/**
 * A velocity obstacle.
 */
class VelocityObstacle
{
   public:
    /**
     * Construct a velocity obstacle given two sides and an apex. The sides will be
     * normalized to unit vectors.
     *
     * @param apex	the velocity of the obstacle (relative to ground)
     * @param side1	one side of the velocity obstacle (relative to apex)
     * @param side2	one side of the velocity obstacle (relative to apex)
     *
     * @return	velocity obstacle with respective apex and normalized unit-vector sides
     */
    VelocityObstacle(Vector apex, Vector side1, Vector side2);

	/**
	 * Construct a velocity obstacle given the obstacle from the perspective of a robot.
	 *
	 * @param obstacle			the obstacle to consider when constructing the velocity obstacle
	 * @param robot				the robot to consider the obstacle from
	 * @param obstacle_velocity	the velocity of the obstacle
	 *
	 * @return velocity obstacle with respective apex and normalized unit-vector sides constructing an obstacle from the 
	 * 			perspective of a robot
	 */
	static VelocityObstacle generateVelocityObstacle(const Circle& obstacle, const Circle &robot, const Vector& obstacle_velocity);

    /**
     * Getters for the unit-vectors assigned as the right and left sides of the velocity
     * obstacle.
     *
     * @return unit vector representing the sides of the velocity obstacle
     */
    Vector getRightSide() const;
    Vector getLeftSide() const;

    /**
     * Get the apex of the velocity obstacle. Represents the velocity of the obstacle (if
     * we considered it as a point) relative to the ground.
     *
     * @return vector representing the velocity of this velocity obstacle
     */
    Vector getApex() const;

    /**
     * Returns true if the given velocity is inside this VelocityObstacle, false
     * otherwise.
     *
     * @param velocity the velocity to compare
     *
     * @return bool true if this VelocityObstacle contains the given velocity, false
     * otherwise
     */
    bool containsVelocity(const Vector &velocity) const;

   private:
    // The position of the apex of the hybrid reciprocal velocity obstacle.
    Vector apex;

    // The direction of t(he right side of the velocity obstacle (in the frame of
    // reference of the velocity obstacle).
    Vector right_side;

    // The direction of the left side of the velocity obstacle in the frame of reference
    // of the velocity obstacle.
    Vector left_side;
};
