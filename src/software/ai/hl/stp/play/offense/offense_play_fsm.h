#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"
#include "software/logger/logger.h"

struct OffensePlayFSM
{
    class OffensiveState;
    class DefensiveState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates an offense play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit OffensePlayFSM(
        TbotsProto::AiConfig ai_config,
        std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    /**
     * Guard to check whether the enemy team has possession of the ball
     *
     * @param event the FSM event
     *
     * @return whether the enemy team has possession of the ball
     */
    bool enemyHasPossession(const Update& event);

    /**
     * Action to configure the play for offensive gameplay
     *
     * @param event the FSM event
     */
    void setupOffensiveStrategy(const Update& event);

    /**
     * Action to configure the play for defensive gameplay
     *
     * @param event the FSM event
     */
    void setupDefensiveStrategy(const Update& event);

    /**
     * Helper function to set the tactics for the play depending on the
     * specified number of attackers and defenders to setup
     *
     * @param event the FSM event
     * @param num_shoot_or_pass the number of attackers (ShootOrPassPlay)
     * @param num_defenders the number of defenders (DefensePlay)
     */
    void setTactics(const Update& event, int num_shoot_or_pass, int num_defenders);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(OffensiveState)
        DEFINE_SML_STATE(DefensiveState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(enemyHasPossession)

        DEFINE_SML_ACTION(setupOffensiveStrategy)
        DEFINE_SML_ACTION(setupDefensiveStrategy)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *OffensiveState_S + Update_E[enemyHasPossession_G] /
                                    setupDefensiveStrategy_A       = DefensiveState_S,
            OffensiveState_S + Update_E / setupOffensiveStrategy_A = OffensiveState_S,
            DefensiveState_S + Update_E[!enemyHasPossession_G] /
                                   setupOffensiveStrategy_A        = OffensiveState_S,
            DefensiveState_S + Update_E / setupDefensiveStrategy_A = DefensiveState_S,
            X + Update_E                                           = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<ShootOrPassPlay> shoot_or_pass_play;
    std::shared_ptr<DefensePlay> defense_play;
};
