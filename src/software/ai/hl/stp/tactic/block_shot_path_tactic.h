#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The BlockShotPathTactic will move the robot to block the entire friendly net from
 * the shot
 */
class BlockShotPathTactic : public Tactic
{
   public:
    /**
     * Creates a new BlockShotPathTactic
     *
     * @param field the Field we are playing on
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit BlockShotPathTactic(const Field& field, bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the control parameters for this BlockShotPathTactic.
     *
     * @param enemy_robot The enemy robot to block from shooting on the friendly net
     */
    void updateControlParams(const Robot& enemy_robot);

    /**
     * Updates the control parameters for this BlockShotPathTactic.
     *
     * @param shot_origin The origin of the shot to block
     */
    void updateControlParams(const Point& shot_origin);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    void accept(TacticVisitor& visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    /**
     * Calculates the location to move to in order to block the shot.
     * @return the Point to move to in order to block the shot
     */
    Point getBlockPosition();

    // Tactic parameters
    // The point the shot to block will be starting from
    Point shot_origin;
    // The field we are playing on
    Field field;
};
