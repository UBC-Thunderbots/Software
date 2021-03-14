#pragma once

#include "software/ai/intent/navigating_intent.h"

class MoveIntent : public NavigatingIntent
{
   public:
    /**
     * Creates a new Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param destination The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param dribbler_mode Dribbler mode
     * @param ball_collision_type how to navigate around the ball
     * @param auto_chip_or_kick The command to autochip or autokick
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     * @param spin_speed The angular velocity of fixed spin speed while moving
     */
    explicit MoveIntent(unsigned int robot_id, const Point& destination,
                        const Angle& final_angle, double final_speed,
                        const DribblerMode& dribbler_mode,
                        const BallCollisionType& ball_collision_type,
                        const AutoChipOrKick& auto_chip_or_kick,
                        const MaxAllowedSpeedMode& max_allowed_speed_mode,
                        const AngularVelocity& spin_speed);

    MoveIntent() = delete;

    void accept(IntentVisitor& visitor) const override;
    void accept(NavigatingIntentVisitor& visitor) const override;

    /**
     * Gets the robot's destination orientation
     *
     * @return The robots final orientation as an Angle
     */
    const Angle& getFinalAngle() const;

    /**
     * Gets DribblerMode for this move intent
     *
     * @return dribbler mode
     */
    const DribblerMode& getDribblerMode() const;

    /**
     * Gets the AutoChipOrKick for this move intent
     *
     * @return the chip kick command
     */
    const AutoChipOrKick& getAutoChipOrKick() const;

    /**
     * Gets the spin speed for this move intent
     *
     * @return the spin speed
     */
    const AngularVelocity& getSpinSpeed() const;

    /**
     * Compares MoveIntents for equality. MoveIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the MoveIntent to compare with for equality
     *
     * @return true if the MoveIntents are equal and false otherwise
     */
    bool operator==(const MoveIntent& other) const;

    /**
     * Compares MoveIntents for inequality.
     *
     * @param other the MoveIntent to compare with for inequality
     *
     * @return true if the MoveIntents are not equal and false otherwise
     */
    bool operator!=(const MoveIntent& other) const;

   private:
    Angle final_angle;
    DribblerMode dribbler_mode;
    AutoChipOrKick auto_chip_or_kick;
    AngularVelocity spin_speed;
};
