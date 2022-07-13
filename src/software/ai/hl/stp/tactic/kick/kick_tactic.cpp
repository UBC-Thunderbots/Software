#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

KickTactic::KickTactic()
    : Tactic({RobotCapability::Kick, RobotCapability::Move}), fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<KickFSM>>(GetBehindBallFSM());
    }
}

void KickTactic::updateControlParams(const Point &kick_origin,
                                     const Angle &kick_direction,
                                     AutoChipOrKick auto_chip_or_kick)
{
    control_params.kick_origin       = kick_origin;
    control_params.kick_direction    = kick_direction;
    control_params.auto_chip_or_kick = auto_chip_or_kick;
}

void KickTactic::updateControlParams(const Point &kick_origin, const Point &kick_target,
                                     AutoChipOrKick auto_chip_or_kick)
{
    updateControlParams(kick_origin, (kick_target - kick_origin).orientation(),
                        auto_chip_or_kick);
}

void KickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void KickTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<KickFSM>>(GetBehindBallFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(KickFSM::Update(control_params, tactic_update));
}
