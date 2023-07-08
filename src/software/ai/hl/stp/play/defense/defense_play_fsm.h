#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/logger/logger.h"
#include "software/ai/evaluation/defender_assignment.h"

struct DefensePlayFSM
{
    class DefenseState;
    class AggressiveDefenseState;

    struct ControlParams
    {
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;

        // The defender assignments, ordered such that the assignments with the 
        // highest coverage ratings at the front of the queue
        std::queue<DefenderAssignment> defender_assignments;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Distance away from an enemy that shadowers will position themselves to shadow
    const double ROBOT_SHADOWING_DISTANCE_METERS = ROBOT_MAX_RADIUS_METERS * 3;

    /**
     * Creates a defense play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit DefensePlayFSM(TbotsProto::AiConfig ai_config);

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
     * defenders to block enemy shots and passes
     *
     * @param event the FSM event
     */
    void shadowAndBlockShots(const Update& event);

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

    /**
     * Helper function to reset the crease and pass defender assignment vectors
     * and recompute all enemy threats
     *
     * @param event the FSM event
     */    
    void resetAssignmentsAndEnemyThreats(const Update& event);

    /**
     * Helper function to assign one crease defender using the defender assignment
     * with the highest coverage rating
     *
     * @param event the FSM event
     * @param num_tactics_available the number of tactics left available to assign
     *
     * @return the number of crease defenders assigned
     */
    int assignPrimaryCreaseDefender(const Update &event, int num_tactics_available);

    /**
     * Helper function to assign crease and pass defenders based on
     * the number of remaining tactics left to assign
     *
     * @param event the FSM event
     * @param num_tactics_to_assign the number of tactics to try to assign
     *
     * @return the number of tactics assigned
     */
    int updateCreaseAndPassDefenders(const Update& event, const int num_tactics_to_assign);

    /**
     * Helper function to assign shadowers based on the number of remaining 
     * tactics left to assign
     *
     * @param event the FSM event
     * @param num_shadowers_to_assign the number of shadowers to try to assign
     *
     * @return the number of shadowers assigned
     */
    int updateShadowers(const Update& event, const int num_shadowers_to_assign);

    /**
     * Helper function to add a shadow enemy tactic that will shadow the 
     * given enemy threat
     *
     * @param enemy_threat the enemy threat to shadow
     */
    void addShadower(EnemyThreat &enemy_threat);

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

    /**
     * Helper function to return the next tactics
     *
     * @param event the FSM event
     */
    void setTactics(const Update& event);

    TbotsProto::AiConfig ai_config;

    std::vector<EnemyThreat> enemy_threats;
    
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
    std::vector<std::shared_ptr<PassDefenderTactic>> pass_defenders;
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadowers;

    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;

    std::queue<DefenderAssignment> defender_assignments_queue;

    std::optional<DefenderAssignment> highest_cov_rating_assignment;
};
