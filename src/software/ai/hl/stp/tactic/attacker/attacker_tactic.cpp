#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(
    std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config)
    : Tactic(false,
             {RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      fsm(DribbleFSM(std::make_shared<Point>())),
      best_pass_so_far(std::nullopt),
      pass_committed(false),
      chip_target(std::nullopt),
      attacker_tactic_config(attacker_tactic_config)
{
}

void AttackerTactic::updateWorldParams(const World& world) {}

void AttackerTactic::updateControlParams(const Pass& best_pass_so_far,
                                         bool pass_committed)
{
    // Update the control parameters stored by this Tactic
    this->best_pass_so_far = best_pass_so_far;
    this->pass_committed   = pass_committed;
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
        .best_pass_so_far       = best_pass_so_far,
        .pass_committed         = pass_committed,
        .shot                   = shot,
        .chip_target            = chip_target,
        .attacker_tactic_config = attacker_tactic_config};

    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}

bool AttackerTactic::done() const
{
    return fsm.is(boost::sml::X);
}

double AttackerTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // Default 0 cost assuming ball is in dribbler
    double cost = 0.0;
    if (!robot.isNearDribbler(world.ball().position()))
    {
        // Prefer robots closer to the interception point. If no interception point
        // exists, prefer robots closer to ball We normalize with the total field length
        // so that robots that are within the field have a cost less than 1

        auto intercept_result =
            findBestInterceptForBall(world.ball(), world.field(), robot)
                .value_or(std::make_pair(world.ball().position(), Duration()));

        cost = (robot.position() - intercept_result.first).length() /
               world.field().totalXLength();
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

std::string AttackerTactic::getAdditionalInfo() const
{
    std::stringstream ss;
    fsm.visit_current_states([&ss](auto state) { ss << TYPENAME(state); });
    return parseCondensedFsmState(ss.str());
}
