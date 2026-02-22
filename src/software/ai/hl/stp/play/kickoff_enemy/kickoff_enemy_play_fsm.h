#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/logger/logger.h"



struct KickoffEnemyPlayFSM: PlayFSM<KickoffEnemyPlayFSM>
{
    class SetupState;

    struct ControlParams
    {
    };


    /**
     * Creates a kickoff enemy play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit KickoffEnemyPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);


    /**
     * create a vector of setup positions if not already existing.
     *
     * @param world_ptr the world pointer
     */
    void createKickoffSetupPositions(const WorldPtr &world_ptr);

    /**
     * add shadowing robots to tactics to run.
     *
     * @param enemy_threats the enemies that must be shadowed.
     * @param tactics_to_run vector of tactics to run.
     * @param defense_position_index index of robot for priority.
     */
    void assignShadowing(const std::vector<EnemyThreat> &enemy_threats,
                         PriorityTacticVector &tactics_to_run,
                         size_t &defense_position_index);

    /**
     * add defenders to tactics to run.
     *
     * @param tactics_to_run vector of tactics to run.
     * @param defense_position_index index of robot for priority.
     */
    void assignDefenders(PriorityTacticVector &tactics_to_run,
                         size_t &defense_position_index);

    /**
     * add a goal blocker to tactics to run.
     *
     * @param world_ptr the world pointer
     * @param tactics_to_run vector of tactics to run.
     * @param defense_position_index index of robot for priority.
     */
    void assignGoalBlocker(const WorldPtr &world_ptr,
                           PriorityTacticVector &tactics_to_run,
                           size_t &defense_position_index);

    /**
     * Action to organize the bots to be ready for enemy kickoff.
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void kickoff(const Update &event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(kickoff)


        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            // PlaySelectionFSM will transition to OffensePlay after the kick.
            *SetupState_S + Update_E / kickoff_A = SetupState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_enemy_tactics;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::vector<Point> kickoff_setup_positions;
};
