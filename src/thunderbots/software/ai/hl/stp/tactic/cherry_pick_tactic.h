/**
 * Declaration for the CherryPickTactic class
 */
#pragma once

#include <ai/passing/pass_generator.h>

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/world/world.h"
#include "geom/rectangle.h"

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
    explicit CherryPickTactic(const World& world, const Rectangle& target_region,
                              bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this tactic
     *
     * @param world The current state of the world
     */

    void updateParams(const World& world);

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

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // The region in which we want to position the cherry picking robot
    Rectangle target_region;

    // The pass optimizer being used to figure out the best position for the robot
    AI::Passing::PassGenerator pass_generator;

    // Tactic parameters
    // The current state of the world
    World world;
};
