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
     * @param autochick_command The command to autochip or autokick
     * @param max_speed_m_per_s The maximum speed in meters per second
     */
    explicit MoveIntent(
        unsigned int robot_id, const Point& destination, const Angle& final_angle,
        double final_speed, DribblerMode dribbler_mode,
        BallCollisionType ball_collision_type,
        std::optional<TbotsProto::AutochickCommand> autochick_command = std::nullopt,
        double max_speed_m_per_s = ROBOT_MAX_SPEED_METERS_PER_SECOND);

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
     * Gets the AutochickCommand for this move intent
     *
     * @return the autochick command
     */
    std::optional<TbotsProto::AutochickCommand> getAutochickCommand() const;

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
    std::optional<TbotsProto::AutochickCommand> autochick_command;
};
