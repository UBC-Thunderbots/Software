#pragma once

/**
 * Class to represent the kinematic constraints of a robot
 */
class KinematicConstraints
{
   public:
    KinematicConstraints() = delete;

    /**
     * Constructor
     * @param max_velocity The max velocity a robot can have
     * @param max_acceleration The max acceleration a robot can have
     * @param max_deceleration The max deceleration a robot can have
     */
    inline KinematicConstraints(double max_velocity, double max_acceleration,
                                double max_deceleration)
        : max_velocity(max_velocity),
          max_acceleration(max_acceleration),
          max_deceleration(max_deceleration)
    {
    }

    /**
     * Get the max velocity
     * @return max velocity of this kinematic constraint
     */
    double getMaxVelocity() const
    {
        return max_velocity;
    }

    /**
     * Get the max acceleration
     * @return max acceleration of this kinematic constraint
     */
    double getMaxAcceleration() const
    {
        return max_acceleration;
    }

    /**
     * Get the max deceleration
     * @return max deceleration of this kinematic constraint
     */
    double getMaxDeceleration() const
    {
        return max_deceleration;
    }

   private:
    double max_velocity;
    double max_acceleration;
    double max_deceleration;
};
