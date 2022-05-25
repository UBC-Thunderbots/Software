#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_tactic.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

PivotKickTactic::PivotKickTactic(std::shared_ptr<const AiConfig> ai_config)
    : Tactic({RobotCapability::Move, RobotCapability::Kick, RobotCapability::Chip,
              RobotCapability::Dribble}),
      fsm_map(),
      control_params(PivotKickFSM::ControlParams()),
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PivotKickFSM>>(
            DribbleFSM(ai_config->getDribbleTacticConfig()));
    }
}

void PivotKickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PivotKickTactic::updateControlParams(const Point &kick_origin,
                                          const Angle &kick_direction,
                                          AutoChipOrKick auto_chip_or_kick)
{
    control_params.kick_origin       = kick_origin;
    control_params.kick_direction    = kick_direction;
    control_params.auto_chip_or_kick = auto_chip_or_kick;
}

void PivotKickTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<PivotKickFSM>>(
            DribbleFSM(ai_config->getDribbleTacticConfig()));
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(PivotKickFSM::Update(control_params, tactic_update));
}
