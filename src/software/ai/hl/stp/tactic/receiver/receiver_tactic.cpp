#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(const Pass pass)
    : Tactic(false, {RobotCapability::Move}),
      pass(std::move(pass)),
      fsm(ReceiverFSM(std::make_shared<Timestamp>())),
      control_params(
          {ReceiverFSM::ControlParams{.pass = std::nullopt, .disable_one_touch = false}})
{
}

void ReceiverTactic::updateWorldParams(const World& world) {}
void ReceiverTactic::updateControlParams(const Pass& updated_pass, bool disable_one_touch)
{
    // Update the control parameters stored by this Tactic
    control_params.pass              = updated_pass;
    control_params.disable_one_touch = disable_one_touch;
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
    // Prefer robots closer to the pass receive position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - pass.receiverPoint()).length() / world.field().totalXLength();

    // TODO (#2167) robocup 2021 hack: prevents oscillating tactic assignments that give
    // up the ball
    //
    // The attacker and receiver are the two tactics that need the ball/try to get the
    // ball. We want these tactics to be the most expensive, so that the munkres algorithm
    // minimizes the overal cost by assinging these tactics to the robots nearest to the
    // ball.
    return std::clamp<double>(cost, 0, 1) * 10;
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

std::string ReceiverTactic::getAdditionalInfo() const
{
    std::stringstream ss;
    fsm.visit_current_states([&ss](auto state) { ss << TYPENAME(state); });
    return parseCondensedFsmState(ss.str());
}
