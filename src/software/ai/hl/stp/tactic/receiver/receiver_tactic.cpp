#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic()
    : Tactic(false, {RobotCapability::Move}),
      fsm(ReceiverFSM()),
      control_params({ReceiverFSM::ControlParams{.pass                   = std::nullopt,
                                                 .disable_one_touch_shot = false}})
{
}

void ReceiverTactic::updateWorldParams(const World& world) {}
void ReceiverTactic::updateControlParams(const Pass& updated_pass,
                                         bool disable_one_touch_shot)
{
    // Update the control parameters stored by this Tactic
    control_params.pass                   = updated_pass;
    control_params.disable_one_touch_shot = disable_one_touch_shot;
}

void ReceiverTactic::updateIntent(const TacticUpdate& tactic_update)
{
    fsm.process_event(ReceiverFSM::Update(control_params, tactic_update));
}

bool ReceiverTactic::done() const
{
    return fsm.is(boost::sml::X);
}

double ReceiverTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // If we have no receive point, make it expensive to assign this robot
    if (!control_params.pass.has_value())
    {
        return 1;
    }

    // Prefer robots closer to the pass receive position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - control_params.pass->receiverPoint()).length() /
                  world.field().totalXLength();

    return std::clamp<double>(cost, 0, 1);
}

void ReceiverTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    auto stop_action = std::make_shared<StopAction>(true);
    while (true)
    {
        yield({stop_action});
    }
}

void ReceiverTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
