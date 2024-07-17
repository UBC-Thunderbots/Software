#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/defense/defense_play_base.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

/**
 * Play for defending against enemy free kicks
 */
struct EnemyFreeKickPlayFSM : public DefensePlayFSMBase
{
    class BlockEnemyKickerState;

    static constexpr double TOO_CLOSE_THRESHOLD_METERS = 0.1;

    /**
     * Creates a enemy free kick play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit EnemyFreeKickPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to configure the play for defensive gameplay for enemy free kick
     *
     * @param event the FSM event
     */
    void blockEnemyKicker(const Update &event);

    /**
     * Helper function to set the tactics for the play depending on the
     * specified number of attackers and defenders to setup
     *
     * @param event the FSM event
     * @param num_tactics the number of tactics available to assign
     */
    void setTactics(const Update &event, unsigned int num_tactics);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(BlockEnemyKickerState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(blockEnemyKicker)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *BlockEnemyKickerState_S + Update_E / blockEnemyKicker_A =
                BlockEnemyKickerState_S,
            X + Update_E = X);
    }
};
