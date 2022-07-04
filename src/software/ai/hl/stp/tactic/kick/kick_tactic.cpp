#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

KickTactic::KickFSM(TbotsProto::KickTacticConfig kick_tactic_config)
: kick_tactic_config(kick_tactic_config)
{

}

KickTactic::KickTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Kick, RobotCapability::Move}), fsm_map(), ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<KickFSM>>(GetBehindBallFSM());
    }
}

void KickTactic::updateControlParams(const Point &kick_origin,
                                     const Angle &kick_direction,
                                     double kick_speed_meters_per_second)
{
    control_params.kick_origin                  = kick_origin;
    control_params.kick_direction               = kick_direction;
    control_params.kick_speed_meters_per_second = kick_speed_meters_per_second;
}

void KickTactic::updateControlParams(const Point &kick_origin, const Point &kick_target,
                                     double kick_speed_meters_per_second)
{
    updateControlParams(kick_origin, (kick_target - kick_origin).orientation(),
                        kick_speed_meters_per_second);
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
