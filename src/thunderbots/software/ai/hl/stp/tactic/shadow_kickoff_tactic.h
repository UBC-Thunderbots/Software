#pragma once

#include "ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowKickoffTactic will move the assigned robot to shadow an enemy
 * robot during kick off. The robot that is the closest to an enemy robot that
 * will shadow the robot and make sure that it is nearby to intercept any action
 * that the enemy may perform.
 */
class ShadowKickoffTactic : public Tactic
{
   public:
    /**
     * Creates a new ShadowKickoffTactic
     *
     * @param field The field the kickoff is running on
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes. ShadowKickoffTactic defaults this
     * to true as the enemy robot position will be constantly updating and will require
     * constant correction.
     */
    explicit ShadowKickoffTactic(const Field& field, bool loop_forever = true);

    std::string getName() const override;

    /**
     * Updates the parameters for this ShadowKickoffTactic.
     *
     * @param enemy_robot_position The enemy robot to shadow without moving to the enemy
     * side.
     */
    void updateParams(Point enemy_robot_position);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the enemy robot
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    /*
     * Calculates the point which the robot needs to position it self on the field to
     * shadow the specified enemy robot. Takes into account the field lines, and does not
     * return points in the centre circle, complying with the rules of the kickoff.
     *
     * @param field The field that the tactic needs to run on
     * @param enemy_robot_position The position of the enemy robot to defend
     * @return The destination point, which is the best point to shadow the specified
     * enemy robot
     */
    Point calculateShadowPoint(const Field& field, const Point& enemy_robot_position);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic parameters
    // Where the enemy robot is located
    Point enemy_robot_position;
    // The current field where the robots are
    Field field;
};
