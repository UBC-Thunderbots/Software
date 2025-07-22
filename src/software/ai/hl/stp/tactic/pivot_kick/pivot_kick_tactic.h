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

    /**
     * Update control params for this tactic
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param auto_chip_or_kick The command to autochip or autokick
     */
    void updateControlParams(const Point& kick_origin, const Angle& kick_direction,
                             AutoChipOrKick auto_chip_or_kick);

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<PivotKickFSM>> fsm_init() override;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    PivotKickFSMControlParams control_params;

};

COPY_TACTIC(WallKickoffTactic, PivotKickTactic)
