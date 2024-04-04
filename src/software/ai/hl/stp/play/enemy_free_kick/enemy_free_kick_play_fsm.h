#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
//#include "software/ai/hl/stp/play/defense/defense_play.h"

#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

/**
 * Play for defending against enemy free kicks
 */

struct EnemyFreeKickPlayFSM {
    class BlockFreeKicker;

    struct ControlParams {
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
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
    void setupEnemyKickerStrategy(const Update &event);

    /**
     * Helper function to set the tactics for the play depending on the
     * specified number of attackers and defenders to setup
     *
     * @param event the FSM event
     * @param num_tactics the number of tactics available to assign
     */
    void setTactics(const Update &event, unsigned int num_tactics);

    /**
     * Helper function to set up crease defender tactic vector members
     *
     * @param num_crease_defenders the number of crease defender tactics to set
     */
    void setUpCreaseDefenders(unsigned int num_crease_defenders);

    /**
     * Helper function to set up pass defender tactic vector members
     *
     * @param num_pass_defenders the number of pass defender tactics to set
     */
    void setUpPassDefenders(unsigned int num_pass_defenders);

    auto operator()() {
        using namespace boost::sml;

        DEFINE_SML_STATE(BlockFreeKicker)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupEnemyKickerStrategy)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *BlockFreeKicker_S + Update_E / setupEnemyKickerStrategy_A = BlockFreeKicker_S,
                X + Update_E = X);
    }

private:
    TbotsProto::AiConfig ai_config;
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
    std::vector<std::shared_ptr<PassDefenderTactic>> pass_defenders;
};
