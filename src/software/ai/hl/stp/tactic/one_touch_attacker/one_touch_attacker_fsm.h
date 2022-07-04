#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

struct OneTouchAttackerFSM
{
    class KickState;
    class AlignToBallState;


    /**
     * Constructor for OneTouchAttackerFSM
     *
     * @param attacker_tactic_config The config to fetch parameters from
     */
    explicit OneTouchAttackerFSM(TbotsProto::AttackerTacticConfig attacker_tactic_config)
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
        // Whether we should keep away
        bool should_keep_away = true;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the PivotKickFSM to shoot or pass
     *
     * @param event OneTouchAttackerFSM::Update event
     * @param processEvent processes the PivotKickFSM::Update
     */
    void pivotKick(const Update& event,
                   boost::sml::back::process<PivotKickFSM::Update> processEvent);

    /**
     * Action that updates the DribbleFSM to keep the ball away
     *
     * @param event OneTouchAttackerFSM::Update event
     * @param processEvent processes the DribbleFSM::Update
     */
    void keepAway(const Update& event,
                  boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * Guard that checks if the ball should be kicked, which is when there's a nearby
     * enemy or a good pass/shot
     *
     * @param event OneTouchAttackerFSM::Update event
     *
     * @return if the ball should be kicked
     */
    bool shouldKick(const Update& event);

    void kickBall(const Update& event);
    void alignToBall(const Update& event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(KickFSM)
        DEFINE_SML_STATE(AlignToBallState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldKick)
        DEFINE_SML_ACTION(alignToBall)
        DEFINE_SML_ACTION(kickBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *AlignToBallState_S + Update_E[shouldKick_G] / kickBall_A = KickState_S,
            AlignToBallState_S + Update_E[!shouldKick_G] / alignToBall_A,
            KickState_S + Update_E / kickBall_A, KickState_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    // the attacker tactic config
    TbotsProto::AttackerTacticConfig attacker_tactic_config;
};
