#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<GetBehindBallFSM>({RobotCapability::Move}, ai_config_ptr),
      control_params({.ball_location = Point(0, 0), .chick_direction = Angle::zero()})
{
}

void GetBehindBallTactic::updateControlParams(const Point &ball_location,
                                              Angle chick_direction)
{
    control_params.ball_location   = ball_location;
    control_params.chick_direction = chick_direction;
}

void GetBehindBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void GetBehindBallTactic::updatePrimitive(const TacticUpdate &tactic_update,
                                          bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(GetBehindBallFSM::Update(control_params, tactic_update));
}
