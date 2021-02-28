#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass_generator.h"
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
     */
    explicit CherryPickTactic(const World& world, const Rectangle& target_region);

    CherryPickTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic.
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;
    /**
     *
     * @return
     */
    World getWorld() const;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
};
