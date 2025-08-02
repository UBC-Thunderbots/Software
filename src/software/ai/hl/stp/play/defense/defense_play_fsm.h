#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/defense/defense_play_base.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/logger/logger.h"

struct DefensePlayFSM : public DefensePlayFSMBase
{
    class DefenseState;
    class AggressiveDefenseState;
    /**
     * Creates a defense play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit DefensePlayFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Guard to check whether we should be defending more aggressively
     *
     * @param event the FSM event
     *
     * @return whether we should be defending more aggressively
     */
    bool shouldDefendAggressively(const Update& event);

    /**
     * Guard to check whether we should be defending more aggressively
     *
     * @param event the FSM event
     *
     * @return whether we should be defending more aggressively
     */
    bool shouldDefendAggressively(const Update& event);

    /**
     * Action to identify all immediate enemy threats and assign
     * defenders to block enemy shots and passes
     *
     * @param event the FSM event
     */
    void blockShots(const Update& event);

    /**
     * Action to shadow the primary enemy threat and assign
     * extra defenders to block enemy shots and passes
     *
     * @param event the FSM event
     */
    void shadowAndBlockShots(const Update& event);

    /**
     * Helper function to update crease and pass defenders to defend
     * against specified threats
     *
     * @param event the FSM event
     * @param enemy_threats the enemy threats to defend against
     */
    void updateCreaseAndPassDefenders(const Update& event,
                                      const std::vector<EnemyThreat>& enemy_threats);

    /**
     * Helper function to update shadowers to shadow specified threats
     *
     * @param event the FSM event
     * @param threats_to_shadow the enemy threats to shadow
     */
    void updateShadowers(const Update& event,
                         const std::vector<EnemyThreat>& threats_to_shadow);


    /**
     * Helper function to set up shadow enemy tactic vector members
     *
     * @param num_shadowers the number of shadow enemy tactics to set
     */
    void setUpShadowers(int num_shadowers);

    /**
     * Helper function to return the next tactics
     *
     * @param event the FSM event
     */
    void setTactics(const Update& event);



    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DefenseState)
        DEFINE_SML_STATE(AggressiveDefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldDefendAggressively)

        DEFINE_SML_ACTION(blockShots)
        DEFINE_SML_ACTION(shadowAndBlockShots)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state

            *DefenseState_S + Update_E[shouldDefendAggressively_G] /
                                  shadowAndBlockShots_A = AggressiveDefenseState_S,
            DefenseState_S + Update_E / blockShots_A    = DefenseState_S,
            AggressiveDefenseState_S +
                Update_E[!shouldDefendAggressively_G] / blockShots_A = DefenseState_S,
            AggressiveDefenseState_S + Update_E / shadowAndBlockShots_A =
                AggressiveDefenseState_S,
            X + Update_E = X);
    }

   private:
    // Where to shadow, gathered experimentally after seeing
    // how far is far enough that the robot couldn't get dribbled past
    static constexpr double ROBOT_SHADOWING_DISTANCE_METERS = 0.36;
};
