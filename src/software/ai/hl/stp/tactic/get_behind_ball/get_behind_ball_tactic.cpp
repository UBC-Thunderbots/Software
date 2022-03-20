#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic()
    : Tactic({RobotCapability::Move}),
      fsm{GetBehindBallFSM()},
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

double GetBehindBallTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the
    // field have a cost less than 1
    double cost = (robot.position() - control_params.ball_location).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
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
    fsm.process_event(GetBehindBallFSM::Update(control_params, tactic_update));
}
