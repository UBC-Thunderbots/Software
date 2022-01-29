#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

KickTactic::KickTactic()
    : Tactic({RobotCapability::Kick, RobotCapability::Move}), fsm{GetBehindBallFSM()}
{
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

double KickTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // the closer the robot is to a ball, the cheaper it is to perform the kick
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();

    return std::clamp<double>(cost, 0, 1);
}

void KickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void KickTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(KickFSM::Update(control_params, tactic_update));
}
