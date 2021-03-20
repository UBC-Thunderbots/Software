#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick_tactic/penalty_kick_tactic_fsm.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/logger/logger.h"


/**
 * This tactic is for a robot performing a penalty kick.
 */

class PenaltyKickTactic : public Tactic
{
   public:
    /**
     * Creates a new PenaltyKickTactic
     *
     * @param ball : The ball that we're trying to shoot
     * @param field : The field we are playing on
     * @param enemy_goalie : Optional variable for the enemy goalie robot
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit PenaltyKickTactic(const Ball &ball,
                               bool loop_forever);

    PenaltyKickTactic() = delete;

    void updateWorldParams(const World &world) override;
    void updateControlParams();

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) const override;

    bool done() const override;
    void accept(TacticVisitor &visitor) const override;

    Ball getBall() const;
    //Field getField() const;

    /**
     * Helper function that determines whether the shooter robot has a viable shot on net.
     *
     * @return true if the robot has a viable shot and false if the enemy goalkeeper will
     * likely save the shot.
     */
    //bool evaluatePenaltyShot();

    /**
     * Helper function that returns the point on the enemy goal line where the shooter
     * should aim at.
     *
     * @return the Point on the goalie line where the shooter robot should aim
     */
    //Point evaluateNextShotPosition();

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    // Tactic parameters
    Ball ball;
    HFSM<PenaltyKickTacticFSM> fsm;
    PenaltyKickTacticFSM::ControlParams control_params;
};