#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

#include <algorithm>

DribbleTactic::DribbleTactic(std::shared_ptr<const AiConfig> ai_config)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm_map(),
      control_params{DribbleFSM::ControlParams{.dribble_destination       = std::nullopt,
                                               .final_dribble_orientation = std::nullopt,
                                               .allow_excessive_dribbling = false}},
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<DribbleFSM>>(
            DribbleFSM(ai_config->getDribbleTacticConfig()));
    }
}

void DribbleTactic::updateControlParams(std::optional<Point> dribble_destination,
                                        std::optional<Angle> final_dribble_orientation,
                                        bool allow_excessive_dribbling)
{
    control_params.dribble_destination       = dribble_destination;
    control_params.final_dribble_orientation = final_dribble_orientation;
    control_params.allow_excessive_dribbling = allow_excessive_dribbling;
}

void DribbleTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void DribbleTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<DribbleFSM>>(
            DribbleFSM(ai_config->getDribbleTacticConfig()));
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(DribbleFSM::Update(control_params, tactic_update));
}
