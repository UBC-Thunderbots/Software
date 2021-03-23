#pragma once

#include <optional>
#include <unordered_set>

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/rectangle.h"
#include "software/world/world.h"

/**
 * This tactic is intended to place a robot in a given region, and have the robot
 * constantly move towards the best location within that region such that if we were to
 * pass the ball from it's current position, the robot would be in the best possible place
 * to receive it.
 */
class CherryPickTactic : public Tactic
{
   public:
    /**
     * Creates a new CherryPickTactic
     *
     * @param world The world to initialize the cherry picker with
     * @param pass The pass to initialize the cherry picker with
     */
    explicit CherryPickTactic(const World& world, const Pass& pass);

    /**
     * Updates the control parameters for this CherryPickTactic.
     *
     * @param pass The pass to cherry pick near. The play should update
     *             this cherry picker with the pass to move near.
     */
    void updateControlParams(const Pass& pass);
    void updateWorldParams(const World& world) override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;

    World getWorld() const;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;

    // Tactic parameters
    // The current state of the world
    World world_;

    // The pass to go to.
    Pass pass_;
};
