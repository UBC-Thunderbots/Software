#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/segment.h"

class PivotKickTactic : public Tactic
{
   public:
    /**
     * Creates a new PivotKickTactic
     *
     * @param ai_config The AI configuration
     */
    explicit PivotKickTactic(std::shared_ptr<const AiConfig> ai_config);

    PivotKickTactic() = delete;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

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
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<PivotKickFSM>>> fsm_map;

    PivotKickFSM::ControlParams control_params;
    std::shared_ptr<const AiConfig> ai_config;
};
