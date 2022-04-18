#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"


/**
 * This tactic is for a robot performing a penalty kick.
 */

class PenaltyKickTactic : public Tactic
{
   public:
    /**
     * Creates a new PenaltyKickTactic
     *
     * @param ai_config The AI configuration
     */
    explicit PenaltyKickTactic(std::shared_ptr<const AiConfig> ai_config);

    PenaltyKickTactic() = delete;

    void updateControlParams();

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

    void accept(TacticVisitor &visitor) const override;

   private:
    void updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<PenaltyKickFSM>>> fsm_map;
    std::shared_ptr<const AiConfig> ai_config;
};
