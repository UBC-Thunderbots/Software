#pragma once

#include "software/ai/intent/navigating_intent.h"

class AutokickMoveIntent : public NavigatingIntent
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
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     * @param ball_collision_type how to navigate around the ball
     */
    explicit AutokickMoveIntent(unsigned int robot_id, const Point& destination,
                                const Angle& final_angle, double final_speed,
                                DribblerMode dribbler_mode,
                                double kick_speed_meters_per_second,
                                BallCollisionType ball_collision_type);

    AutokickMoveIntent() = delete;

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
     * Gets the kick speed in metres per second
     *
     * @return the kick speed
     */
    double getKickSpeed() const;

    /**
     * Compares AutokickMoveIntents for equality. AutokickMoveIntents are considered equal
     * if all their member variables are equal.
     *
     * @param other the AutokickMoveIntent to compare with for equality
     *
     * @return true if the AutokickMoveIntents are equal and false otherwise
     */
    bool operator==(const AutokickMoveIntent& other) const;

    /**
     * Compares AutokickMoveIntents for inequality.
     *
     * @param other the AutokickMoveIntent to compare with for inequality
     *
     * @return true if the AutokickMoveIntents are not equal and false otherwise
     */
    bool operator!=(const AutokickMoveIntent& other) const;

   private:
    Angle final_angle;
    DribblerMode dribbler_mode;
    double kick_speed_meters_per_second;
};
