#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

/**
 * Struct containing frequently shared functions of the defense play class
 */
class DefensePlayFSMBase
{
   public:
    struct ControlParams
    {
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a play FSM with defensive methods
     *
     * @param ai_config the play config for this play FSM
     */
    explicit DefensePlayFSMBase(TbotsProto::AiConfig ai_config);

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
    void setAlignment(
        const Update &event,
        const std::vector<DefenderAssignment> &crease_defender_assignments,
        TbotsProto::BallStealMode ball_steal_mode = TbotsProto::BallStealMode::STEAL);

    /**
     * Helper function to update all given pass defender control params
     *
     * @param pass_defender_assignments pass defender assignments to be updated
     */
    void updatePassDefenderControlParams(
        std::vector<DefenderAssignment> &pass_defender_assignments);

    TbotsProto::AiConfig ai_config;
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
    std::vector<std::shared_ptr<PassDefenderTactic>> pass_defenders;
};
