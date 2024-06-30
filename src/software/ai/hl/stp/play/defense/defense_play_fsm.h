#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/defense/defense_play_base.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

struct DefensePlayFSM : public DefensePlayFSMBase
{
    class DefenseState;

    /**
     * Creates a defense play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit DefensePlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to identify all immediate enemy threats and assign
     * defenders to block enemy shots and passes
     *
     * @param event the FSM event
     */
    void defendAgainstThreats(const Update& event);

//    /**
//     * Guard that checks if the ball is nearby and unguarded by the enemy
//     *
//     * @param event CreaseDefenderFSM::Update event
//     *
//     * @return if the ball is nearby and unguarded by the enemy
//     */
//    bool ballNearbyWithoutThreat(const Update& event);
//
//    /**
//     * This is the Action that prepares for getting possession of the ball
//     * @param event CreaseDefenderFSM::Update event
//     * @param processEvent processes the DribbleFSM::Update
//     */
//    void prepareGetPossession(const Update& event,
//                              boost::sml::back::process<DribbleFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(defendAgainstThreats)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DefenseState_S + Update_E / defendAgainstThreats_A = DefenseState_S,
            X + Update_E                                        = X);
    }

//    private:
//    /** Max distance ratio between (crease and ball) / (crease and nearest enemy) for
//     * crease to chase ball. Scale from (0, 1) Crease | <-----------------------> Enemy |
//     * <----> Ball                |
//     *  ()         o                 ()
//     */
//    static constexpr double MAX_GET_BALL_RATIO_THRESHOLD = 0.3;
//    // Max distance that the crease will try and get possession of a ball
//    static constexpr double MAX_GET_BALL_RADIUS_M = 1;
//    // Max speed of ball that crease will try and get possession
//    static constexpr double MAX_BALL_SPEED_TO_GET_MS = 0.5;

};
