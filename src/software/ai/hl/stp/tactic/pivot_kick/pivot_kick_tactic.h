#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/segment.h"

class PivotKickTactic : public Tactic<PivotKickFSM>
{
   public:
    /**
     * Creates a new PivotKickTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PivotKickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    PivotKickTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<PivotKickFSM>> fsm_init() override;
};

COPY_TACTIC(WallKickoffTactic, PivotKickTactic)
