#pragma once

#include "software/ai/intent/navigating_intent.h"
#include "software/primitive/move_primitive.h"

class MoveIntent : public NavigatingIntent
{
   public:
    static const std::string INTENT_NAME;

    /**
     * Creates a new Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param destination The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param slow Whether or not to move slower (1 m/s)
     * @param autokick This will enable the "break-beam" on the robot, that will
     *                        trigger the kicker to fire as soon as the ball is in front
     *                        of it
     * @param ball_collision_type how to navigate around the ball
     */
    explicit MoveIntent(unsigned int robot_id, const Point& destination,
                        const Angle& final_angle, double final_speed,
                        unsigned int priority, DribblerEnable enable_dribbler,
                        MoveType move_type, AutochickType autokick,
                        BallCollisionType ball_collision_type);

    std::string getIntentName(void) const override;
    void accept(IntentVisitor& visitor) const override;
    void accept(NavigatingIntentVisitor& visitor) const override;

    /**
     * Gets the robot's destination orientation
     *
     * @return The robots final orientation as an Angle
     */
    const Angle& getFinalAngle() const;

    /**
     * Gets whether or not auto-kick/auto-chip should be enabled while moving
     *
     * @return whether or not auto-kick/auto-chip should be enabled while moving
     */
    const AutochickType& getAutochickType() const;

    /**
     * Gets whether or not the dribbler should be enabled while moving
     *
     * @return whether or not the dribbler should be enabled while moving
     */
    const DribblerEnable& getDribblerEnable() const;

    /**
     * Gets whether or not the robot should be moving slow
     *
     * @return whether or not the robot should be moving slow
     */
    const MoveType& getMoveType() const;

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
    DribblerEnable enable_dribbler;
    MoveType move_type;
    AutochickType autokick;
};
