#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency


ShadowEnemyTactic::ShadowEnemyTactic()
    :Tactic(false,
            {RobotCapability::Move, RobotCapability::Kick}),
    fsm(),
    control_params{ShadowEnemyFSM::ControlParams{   .enemy_threat = std::nullopt,
                                                    .shadow_distance = 0,
                                                    .get_possession_distance = 0 }}
{
}

void ShadowEnemyTactic::updateWorldParams(const World &world){}

void ShadowEnemyTactic::updateControlParams(std::optional<EnemyThreat> enemy_threat,
                                            double shadow_distance)
{
    control_params.enemy_threat = enemy_threat;
    control_params.shadow_distance = shadow_distance;
}

double ShadowEnemyTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    if (!control_params.enemy_threat)
    {
        return 0;
    }
    // Prefer robots closer to the enemy being shadowed
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - control_params.enemy_threat->robot.position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowEnemyTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
    // auto move_action = std::make_shared<MoveAction>(false, 0, Angle());
    // auto stop_action = std::make_shared<StopAction>(true);

    // do
    // {
    //     if (!enemy_threat)
    //     {
    //         stop_action->updateControlParams(*robot_, false);
    //         yield(stop_action);
    //     }

    //     Robot enemy_robot = enemy_threat->robot;
    //     // If we think the enemy team can pass, and if we have identified a robot that can
    //     // pass to the robot we are shadowing, we block the pass rather than block the
    //     // net. Otherwise, we just block the net
    //   if (enemy_team_can_pass && enemy_threat->passer)
    //     {
    //         Vector enemy_to_passer_vector =
    //             enemy_threat->passer->position() - enemy_robot.position();
    //         Point position_to_block_pass =
    //             enemy_robot.position() +
    //             enemy_to_passer_vector.normalize(this->shadow_distance);
    //         move_action->updateControlParams(*robot_, position_to_block_pass,
    //                                          enemy_to_passer_vector.orientation(), 0,
    //                                          DribblerMode::OFF, BallCollisionType::AVOID);
    //         yield(move_action);
    //     }
    //     else
    //     {
    //         std::vector<Robot> robots_to_ignore = {*robot_};
    //         if (ignore_goalie && friendly_team.goalie())
    //         {
    //             robots_to_ignore.emplace_back(*friendly_team.goalie());
    //         }
    //         auto best_enemy_shot_opt = calcBestShotOnGoal(
    //             field, friendly_team, enemy_team, enemy_robot.position(),
    //             TeamType::FRIENDLY, robots_to_ignore);

    //         Vector enemy_shot_vector = Vector(0, 0);
    //         if (best_enemy_shot_opt)
    //         {
    //             enemy_shot_vector =
    //                 best_enemy_shot_opt->getPointToShootAt() - enemy_robot.position();
    //         }
    //         else
    //         {
    //             enemy_shot_vector = field.friendlyGoalCenter() - enemy_robot.position();
    //         }

    //         Point position_to_block_shot =
    //             enemy_robot.position() +
    //             enemy_shot_vector.normalize(this->shadow_distance);

    //         // If the enemy robot already had the ball, try steal it and chip it away
    //         if (enemy_robot.isNearDribbler(ball.position()) &&
    //             ball.velocity().length() <= ball_steal_speed)
    //         {
    //             move_action->updateControlParams(
    //                 *robot_, ball.position(),
    //                 (ball.position() - robot_->position()).orientation(), 0,
    //                 DribblerMode::MAX_FORCE, BallCollisionType::AVOID,
    //                 {AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS});
    //             yield(move_action);
    //         }
    //         else
    //         {
    //             move_action->updateControlParams(
    //                 *robot_, position_to_block_shot,
    //                 enemy_shot_vector.orientation() + Angle::half(), 0, DribblerMode::OFF,
    //                 BallCollisionType::AVOID);
    //             yield(move_action);
    //         }
    //     }
    // } while (!move_action->done());
}

bool ShadowEnemyTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void ShadowEnemyTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}

void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
