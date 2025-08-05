#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/keep_away/keep_away_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

/**
 * The control parameters for updating AttackerFSM
 */
struct AttackerFSMControlParams
{
    // The best pass so far
    std::optional<Pass> best_pass_so_far;
    // whether we have committed to the pass and will be taking it
    bool pass_committed;
    // The shot to take
    std::optional<Shot> shot;
    // The point the robot will chip towards if it is unable to shoot and is in danger
    // of losing the ball to an enemy
    std::optional<Point> chip_target;
};

struct AttackerFSM : TacticFSM<AttackerFSMControlParams>
{
    using Update = TacticFSM<AttackerFSMControlParams>::Update;
    /**
     * Constructor for AttackerFSM
     *
     * @param ai_config_ptr Shared pointer to ai_config
     */
    explicit AttackerFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
        : TacticFSM<AttackerFSMControlParams>(ai_config_ptr)
    {
    }

    /**
     * Action that updates the PivotKickFSM to shoot or pass
     *
     * @param event AttackerFSM::Update event
     * @param processEvent processes the PivotKickFSM::Update
     */
    void pivotKick(const Update& event,
                   boost::sml::back::process<PivotKickFSM::Update> processEvent);

    /**
     * Action that updates the KeepAwayFSM to keep the ball away
     *
     * @param event AttackerFSM::Update event
     * @param processEvent processes the KeepAwayFSM::Update
     */
    void keepAway(const Update& event,
                  boost::sml::back::process<KeepAwayFSM::Update> processEvent);

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
        DEFINE_SML_STATE(KeepAwayFSM)
        DEFINE_SML_STATE(DribbleFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldKick)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivotKick, PivotKickFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(keepAway, KeepAwayFSM)

        return make_transition_table(
            *DribbleFSM_S + Update_E[shouldKick_G] / pivotKick_A = PivotKickFSM_S,
            DribbleFSM_S + Update_E[!shouldKick_G] / keepAway_A  = KeepAwayFSM_S,
            KeepAwayFSM_S + Update_E[shouldKick_G] / pivotKick_A = PivotKickFSM_S,
            KeepAwayFSM_S + Update_E / keepAway_A, KeepAwayFSM_S    = DribbleFSM_S,
            PivotKickFSM_S + Update_E / pivotKick_A, PivotKickFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
