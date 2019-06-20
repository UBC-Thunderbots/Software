#pragma once

#include "ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is used when performing a free kick.
 * 
 */

class FreeKickTactic : public Tactic
{
    public:
        explicit FreeKickTactic(const World& world, bool loop_forever); 

        std::string getName() const override;

        void updateParams(const World& updated_world);

        
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
        void calculateNextIntent(IntentCoroutine::push_type& yield) override;

        // Tactic params
        World world;
};