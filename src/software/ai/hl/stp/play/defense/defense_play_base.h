#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

/**
 * control parameters for a defense play
 */
struct DefensePlayBaseControlParams
{
    // The maximum allowed speed mode
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
};

/**
 * Struct containing frequently shared functions of the defense play class
 */
class DefensePlayFSMBase : public PlayFSM<DefensePlayBaseControlParams>
{
   public:
    /**
     * Creates a play FSM with defensive methods
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit DefensePlayFSMBase(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

   protected:
    /**
     * Helper function to set up crease defender tactic vector members
     *
     * @param num_crease_defenders the number of crease defender tactics to set
     */
    void setUpCreaseDefenders(int num_crease_defenders);

    /**
     * Helper function to set up pass defender tactic vector members
     *
     * @param num_pass_defenders the number of pass defender tactics to set
     */
    void setUpPassDefenders(int num_pass_defenders);

    /**
     * Helper function to set up alignments for the crease defense robots and control
     * params
     *
     * @param event the FSM event
     * @param crease_defender_assignments crease defender assignments to be aligned
     * @param ball_steal_mode crease defender ball steal behaviour/aggressiveness
     */
    void setAlignment(const Update &event,
                      const std::vector<DefenderAssignment> &crease_defender_assignments,
                      TbotsProto::BallStealMode ball_steal_mode);

    /**
     * Helper function to update all given pass defender control params
     *
     * @param pass_defender_assignments pass defender assignments to be updated
     * @param ball_steal_mode The pass defender's aggressiveness towards the ball
     */
    void updatePassDefenderControlParams(
        std::vector<DefenderAssignment> &pass_defender_assignments,
        TbotsProto::BallStealMode ball_steal_mode);

    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
    std::vector<std::shared_ptr<PassDefenderTactic>> pass_defenders;
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadowers;
};
