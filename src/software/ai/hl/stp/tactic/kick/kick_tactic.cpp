#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/kick_action.h"

KickTactic::KickTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Kick, RobotCapability::Move})
{
}

void KickTactic::updateWorldParams(const World &world) {}

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

void KickTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto kick_action = std::make_shared<KickAction>();
    do
    {
        kick_action->updateControlParams(*robot_, control_params.kick_origin,
                                         control_params.kick_direction,
                                         control_params.kick_speed_meters_per_second);
        yield(kick_action);
    } while (!kick_action->done());
}

void KickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

bool KickTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void KickTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(KickFSM::Update(control_params, tactic_update));
}
