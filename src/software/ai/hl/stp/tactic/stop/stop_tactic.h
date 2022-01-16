#pragma once

#include "software/ai/hl/stp/tactic/stop/stop_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"

/**
 * The StopTactic will stop the robot from moving. The robot will actively try and brake
 * to come to a halt unless is it told to coast, in which case it will coast to a stop.
 */
class StopTactic : public Tactic
{
   public:
    /**
     * Creates a new StopTactic
     t
     * @param coast whether the robot should coast once it's stopped
     */
    explicit StopTactic(bool coast);

    StopTactic() = delete;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;
    bool done() const override;

   private:
    void updateIntent(const TacticUpdate& tactic_update) override;

    FSM<StopFSM> fsm;
};
