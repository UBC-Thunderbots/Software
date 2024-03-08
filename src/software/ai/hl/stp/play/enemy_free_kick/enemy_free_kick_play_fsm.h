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
     * Action to defend the enemy free kick robot
     *
     * @param event the FSM event
     */
    void shadowEnemyKicker(const Update& event);

//    /**
//     * Action to set up the defenders taking into account the new number of defenders
//     *
//     * @param num_defenders The number of defenders
//     */
//    void setUpDefenders(unsigned int num_defenders);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(ShadowFreeKicker)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(shadowEnemyKicker)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *ShadowFreeKicker_S + Update_E / shadowEnemyKicker_A = ShadowFreeKicker_S,
                X + Update_E                                     = X);
    }

private:
    TbotsProto::AiConfig ai_config;
//    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
//    std::shared_ptr<DefensePlay> defense_play;
    std::shared_ptr<TbotsProto::ShadowEnemyTactic> shadow_defender;
};
