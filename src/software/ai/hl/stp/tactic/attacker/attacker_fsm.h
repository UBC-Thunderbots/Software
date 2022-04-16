#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

struct AttackerFSM
{
    /**
     * Constructor for AttackerFSM
     *
     * @param attacker_tactic_config The config to fetch parameters from
     */
    explicit AttackerFSM(
        std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config)
        : attacker_tactic_config(attacker_tactic_config)
    {
    }

    struct ControlParams
    {
        // The best pass so far
        std::optional<Pass> best_pass_so_far = std::nullopt;
        // whether we have committed to the pass and will be taking it
        bool pass_committed = false;
        // The shot to take
        std::optional<Shot> shot = std::nullopt;
        // The point the robot will chip towards if it is unable to shoot and is in danger
        // of losing the ball to an enemy
        std::optional<Point> chip_target;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the PivotKickFSM to shoot or pass
     *
     * @param event AttackerFSM::Update event
     * @param processEvent processes the PivotKickFSM::Update
     */
    void pivotKick(const Update& event,
                   boost::sml::back::process<PivotKickFSM::Update> processEvent);

    /**
     * Action that updates the DribbleFSM to keep the ball away
     *
     * @param event AttackerFSM::Update event
     * @param processEvent processes the DribbleFSM::Update
     */
    void keepAway(const Update& event,
                  boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * Guard that checks if the ball should be kicked, which is when there's a nearby
     * enemy or a good pass/shot
     *
     * @param event AttackerFSM::Update event
     *
     * @return if the ball should be kicked
     */
    bool shouldKick(const Update& event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PivotKickFSM)
        DEFINE_SML_STATE(DribbleFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldKick)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivotKick, PivotKickFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(keepAway, DribbleFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleFSM_S + Update_E[shouldKick_G] / pivotKick_A = PivotKickFSM_S,
            DribbleFSM_S + Update_E[!shouldKick_G] / keepAway_A,
            PivotKickFSM_S + Update_E / pivotKick_A, PivotKickFSM_S = X);
    }

   private:
    // the attacker tactic config
    std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config;
};
