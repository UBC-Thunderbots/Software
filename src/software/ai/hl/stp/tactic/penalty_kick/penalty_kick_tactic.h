#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"


/**
 * This tactic is for a robot performing a penalty kick.
 */

class PenaltyKickTactic
    : public Tactic<PenaltyKickFSM, DribbleFSM, KickFSM, GetBehindBallFSM>
{
   public:
    /**
     * Creates a new PenaltyKickTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PenaltyKickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    PenaltyKickTactic() = delete;

    void updateControlParams();

    void accept(TacticVisitor &visitor) const override;
};
