#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

class MoveAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD       = 0.02;
    static constexpr Angle ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD = Angle::fromDegrees(2);

    /**
     * Creates a new MoveAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit MoveAction(
        bool loop_forever, double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD,
        Angle close_to_orientation_threshold = ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);

    MoveAction() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param dribbler_mode The dribbler mode
     * @param ball_collision_type how to navigate around the ball
     * @param autochipkick The command to autochip or autokick
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     */
    void updateControlParams(
        const Robot& robot, Point destination, Angle final_orientation,
        double final_speed, DribblerMode dribbler_mode,
        BallCollisionType ball_collision_type,
        std::optional<TbotsProto::Autochipkick> autochipkick = std::nullopt,
        MaxAllowedSpeedMode max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    /**
     * Get the destination this MoveAction is going to
     *
     * @return the destination this MoveAction is going to
     */
    Point getDestination();

    /**
     * Get the angle the robot should be at the destination
     *
     * @return the angle the robot should be at the destination
     */
    Angle getFinalOrientation();

    /**
     * Get the speed the robot should be moving at the destination
     *
     * @return the speed the robot should be moving at the destination
     */
    double getFinalSpeed();

    /**
     * Gets the dribbler mode this move action should operate with
     *
     * @return the dribbler mode this move action should operate with
     */
    DribblerMode getDribblerMode();

    /**
     * Gets the Autochipkick for this move intent
     *
     * @return the chip kick command
     */
    std::optional<TbotsProto::Autochipkick> getAutochipkick() const;

   protected:
    /**
     * Checks if robot is close to the destination
     *
     * @return if robot is close to the destination
     */
    bool robotCloseToDestination();

    // Action parameters
    Point destination;
    Angle final_orientation;
    double final_speed;
    DribblerMode dribbler_mode;
    BallCollisionType ball_collision_type;

    double close_to_dest_threshold;
    Angle close_to_orientation_threshold;
    std::optional<TbotsProto::Autochipkick> autochipkick;
    MaxAllowedSpeedMode max_allowed_speed_mode;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;
};
