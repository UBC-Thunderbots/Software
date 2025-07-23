#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"

#include <algorithm>


ChipTactic::ChipTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ChipFSM>({RobotCapability::Chip, RobotCapability::Move}, ai_config_ptr)
{
}

std::unique_ptr<FSM<ChipFSM>> ChipTactic::fsm_init() {
    return std::make_unique<FSM<ChipFSM>>(GetBehindBallFSM(ai_config_ptr), ChipFSM(ai_config_ptr));
}

void ChipTactic::updateControlParams(const Point &chip_origin,
                                     const Angle &chip_direction,
                                     double chip_distance_meters)
{
    control_params.chip_origin          = chip_origin;
    control_params.chip_direction       = chip_direction;
    control_params.chip_distance_meters = chip_distance_meters;
}

void ChipTactic::updateControlParams(const Point &chip_origin, const Point &chip_target)
{
    updateControlParams(chip_origin, (chip_target - chip_origin).orientation(),
                        (chip_target - chip_origin).length());
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void ChipTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(ChipFSM::Update(control_params, tactic_update));
}
