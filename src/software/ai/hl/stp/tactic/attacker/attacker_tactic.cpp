#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"
#include "software/math/math_functions.h"

AttackerTactic::AttackerTactic(
    std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config)
    : Tactic(false,
             {RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      fsm(DribbleFSM(std::make_shared<Point>())),
      pass(std::nullopt),
      chip_target(std::nullopt),
      attacker_tactic_config(attacker_tactic_config)
{
}

void AttackerTactic::updateWorldParams(const World& world) {}

void AttackerTactic::updateControlParams(const std::optional<Pass>& updated_pass)
{
    // Update the control parameters stored by this Tactic
    this->pass = updated_pass;
}

void AttackerTactic::updateControlParams(std::optional<Point> chip_target)
{
    this->chip_target = chip_target;
}

void AttackerTactic::updateIntent(const TacticUpdate& tactic_update)
{
    std::optional<Shot> shot = calcBestShotOnGoal(
        tactic_update.world.field(), tactic_update.world.friendlyTeam(),
        tactic_update.world.enemyTeam(), tactic_update.world.ball().position(),
        TeamType::ENEMY, {tactic_update.robot});
    if (shot && shot->getOpenAngle() <
                    Angle::fromDegrees(
                        attacker_tactic_config->getMinOpenAngleForShotDeg()->value()))
    {
        // reject shots that have an open angle below the minimum
        shot = std::nullopt;
    }

    AttackerFSM::ControlParams control_params{
        .pass                   = pass,
        .shot                   = shot,
        .chip_target            = chip_target,
        .attacker_tactic_config = attacker_tactic_config};

    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}

bool AttackerTactic::done() const
{
    return fsm.is(boost::sml::X);
//    return fsm.is(AttackerFSM::)
//    return fsm.is(boost::sml::X);
//    return fsm.is(AttackerFSM::pivot_kick_s);
}

double AttackerTactic::calculateRobotCost(const Robot& robot, const World& world) const
{ // Default 0 cost assuming ball is in dribbler
    double cost = 0.0;
    if (!robot.isNearDribbler(world.ball().position()))
    {
        // Prefer robots closer to the interception point
        // We normalize with the total field length so that robots that are within the
        // field have a cost less than 1
        cost = (robot.position() -
                DribbleFSM::findInterceptionPoint(robot, world.ball(), world.field()))
                       .length() /
               world.field().totalXLength();
        // If there is already a robot assigned, we strongly prefer to keep that robot
        // if it's roughly near the ball
        if (robot_ && robot_->id() == robot.id())
        {
            const double dist_to_ball =
                    (robot.position() - world.ball().position()).length();
            // Cost is 0 at the ball, stays at zero near the ball, falls
            // away quickly far away from ball
            const double dist_to_ball_cost = sigmoid(dist_to_ball, 1.25, 0.5);
            // If the intercept cost is higher, we use that instead
            // TODO: better comment explaining why
            cost = std::min(cost, dist_to_ball_cost);
        }
    }
    return std::clamp<double>(cost, 0, 1);
}

void AttackerTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    auto stop_action = std::make_shared<StopAction>(false);
    yield({stop_action});
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

bool AttackerTactic::isShooting() {
    return false;
//    return fsm.is(AttackerFSM::pivot_kick_s);
//   fsm.visit_current_states([](auto state) { std::cout << state.c_str() << std::endl; });
//    fsm.visit_current_states([](auto state) { std::cout << TYPENAME(state). << std::endl; });
//    fsm.vi
//    fsm.visit_current_states([](auto state) {
//        std::cout << "state" << std::endl;
//        std::cout << state.c_str() << std::endl;
//    });
//    const auto state_name = state_name_visitor<decltype(fsm)>{fsm};
//    fsm.visit_current_states(state_name);  // s1


//    return "";
}

bool AttackerTactic::isKeepAway() {
//    return fsm.is(AttackerFSM::keep_away_s);
return false;
}

//AttackerFSMStates AttackerTactic::currentFsmState() {
////    return fsm._
//}
