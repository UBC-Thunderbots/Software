#pragma once

#include "software/ai/intent/move_intent.h"

class AutochipMoveIntent : public MoveIntent
{
   public:
    /**
     * Creates a new Autochip Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param destination The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param dribbler_mode Dribbler mode
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     * @param ball_collision_type how to navigate around the ball
     */
    explicit AutochipMoveIntent(unsigned int robot_id, const Point& destination,
                                const Angle& final_angle, double final_speed,
                                DribblerMode dribbler_mode, double chip_distance_meters,
                                BallCollisionType ball_collision_type);

    /**
     * Creates a new Autochip Move Intent
     *
     * @param move_intent The MoveIntent to base this on
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    explicit AutochipMoveIntent(MoveIntent move_intent, double chip_distance_meters);

    AutochipMoveIntent() = delete;

    void accept(IntentVisitor& visitor) const override;
    void accept(NavigatingIntentVisitor& visitor) const override;

    /**
     * Gets the chip distance in metres
     *
     * @return the chip distance
     */
    double getChipDistance() const;

    /**
     * Compares AutochipMoveIntents for equality. AutochipMoveIntents are considered equal
     * if all their member variables are equal.
     *
     * @param other the AutochipMoveIntent to compare with for equality
     *
     * @return true if the AutochipMoveIntents are equal and false otherwise
     */
    bool operator==(const AutochipMoveIntent& other) const;

    /**
     * Compares AutochipMoveIntents for inequality.
     *
     * @param other the AutochipMoveIntent to compare with for inequality
     *
     * @return true if the AutochipMoveIntents are not equal and false otherwise
     */
    bool operator!=(const AutochipMoveIntent& other) const;

   private:
    double chip_distance_meters;
};
