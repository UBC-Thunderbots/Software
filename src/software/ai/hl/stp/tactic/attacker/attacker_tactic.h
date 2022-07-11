#pragma once

#include "software/ai/hl/stp/tactic/attacker/attacker_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

/**
 * This tactic is for a robot performing a pass. It should be used in conjunction with
 * the `ReceiverTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to take the pass as fast as possible
 */
class AttackerTactic : public Tactic
{
   public:
    /**
     * Creates a new AttackerTactic
     *
     * @param ai_config The AI configuration
     */
    explicit AttackerTactic(TbotsProto::AiConfig ai_config);

    AttackerTactic() = delete;

    /**
     * Updates the control parameters for this AttackerTactic.
     *
     * @param updated_pass The pass to perform
     */
    void updateControlParams(const Pass& best_pass_so_far, bool pass_committed);

    /**
     * Updates the control parameters for this AttackerTactic
     *
     * @param chip_target An optional point that the robot will chip towards when it is
     * unable to shoot and is in danger of losing the ball to an enemy. If this value is
     * not provided, the point defaults to the enemy goal
     */
    void updateControlParams(std::optional<Point> chip_target);

    void updateShouldSingleTouch(bool should_single_touch);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<AttackerFSM>>> fsm_map;

    // The pass to execute
    std::optional<Pass> best_pass_so_far;
    // whether we have committed to the above pass
    bool pass_committed;
    // The point the robot will chip towards if it is unable to shoot and is in danger
    // of losing the ball to an enemy
    std::optional<Point> chip_target;

    bool should_single_touch;

    // AI config
    TbotsProto::AiConfig ai_config;
};
