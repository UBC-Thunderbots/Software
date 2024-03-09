#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

/**
 * Play for defending against enemy free kicks
 */

struct EnemyFreeKickPlayFSM
{
    class ShadowFreeKicker;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

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
    void setupEnemyKickerStrategy(const Update& event);

    /**
     * Helper function to set the tactics for the play depending on the
     * specified number of attackers and defenders to setup
     *
     * @param event the FSM event
     * @param num_shadow_robots the number of shadowing robots (ShootOrPassPlay)
     * @param num_defenders the number of defenders (DefensePlay)
     */
    void setTactics(const Update& event, int num_shadow_robots, int num_defenders);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(ShadowFreeKicker)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupEnemyKickerStrategy)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *ShadowFreeKicker_S + Update_E / setupEnemyKickerStrategy_A = ShadowFreeKicker_S,
                X + Update_E                                     = X);
    }

private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<DefensePlay> defense_play;
    std::shared_ptr<TbotsProto::ShadowEnemyTactic> shadow_defender;
};
