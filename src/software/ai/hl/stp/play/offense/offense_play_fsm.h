#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"
#include "software/logger/logger.h"

struct OffensePlayFSM : PlayFSM<OffensePlayFSM>
{
    /**
     * Control Parameters for Offense Play
     */
    struct ControlParams
    {
    };

    class OffensiveState;
    class DefensiveState;

    /**
     * Creates an offense play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit OffensePlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

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

        const auto OffensiveState_S = boost::sml::state<OffensiveState>;
        const auto DefensiveState_S = boost::sml::state<DefensiveState>;

        const auto Update_E = boost::sml::event<Update>;

        const auto enemyHasPossession_G =
            SMLGuard<&OffensePlayFSM::enemyHasPossession>{this};

        const auto setupOffensiveStrategy_A =
            SMLAction<&OffensePlayFSM::setupOffensiveStrategy>{this};
        const auto setupDefensiveStrategy_A =
            SMLAction<&OffensePlayFSM::setupDefensiveStrategy>{this};

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
    std::shared_ptr<ShootOrPassPlay> shoot_or_pass_play;
    std::shared_ptr<DefensePlay> defense_play;
};
