#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic()
    : Tactic({RobotCapability::Move}),
      fsm_map(),
      control_params({.ball_location = Point(0, 0), .chick_direction = Angle::zero()})
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<GetBehindBallFSM>>(GetBehindBallFSM());
    }
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
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<GetBehindBallFSM>>(GetBehindBallFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(GetBehindBallFSM::Update(control_params, tactic_update));
}
