#include <util/parameter/dynamic_parameters.h>
#include "ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/stop_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/robot.h"
#include "shadow_enemy_tactic.h"

DefenseShadowEnemyTactic::DefenseShadowEnemyTactic(const Field &field, const Team &friendly_team,
                                     const Team &enemy_team, const Ball& ball, bool ignore_goalie,
                                     bool loop_forever)
    : field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      shadow_distance(ROBOT_MAX_RADIUS_METERS * 3),
      ignore_goalie(ignore_goalie),
      ball(ball),
      Tactic(loop_forever)
{
}

std::string DefenseShadowEnemyTactic::getName() const
{
    return "Defense Shadow Enemy Tactic";
}

void DefenseShadowEnemyTactic::updateParams(const Evaluation::EnemyThreat &enemy_threat,
                                     const Field &field, const Team &friendly_team,
                                     const Team &enemy_team, double shadow_distance,
                                     const Ball& ball, bool enemy_team_can_pass)
{
    this->enemy_threat        = enemy_threat;
    this->field               = field;
    this->friendly_team       = friendly_team;
    this->enemy_team          = enemy_team;
    this->shadow_distance     = shadow_distance;
    this->ball                = ball;
    this->robot               = robot;
    this->enemy_team_can_pass = enemy_team_can_pass;
}

double DefenseShadowEnemyTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    if (!enemy_threat)
    {
        return 0;
    }
    // Prefer robots closer to the enemy being shadowed
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - enemy_threat->robot.position()).len() /
                  world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

void DefenseShadowEnemyTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction();
    StopAction stop_action =
        StopAction(StopAction::ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT, true);
    do
    {
        if (!enemy_threat)
        {
            yield(stop_action.updateStateAndGetNextIntent(*robot, false));
        }

        Robot enemy_robot = enemy_threat->robot;
        std::vector<Robot> robots_to_ignore = {*robot};

        if (ignore_goalie && friendly_team.goalie())
        {
            robots_to_ignore.emplace_back(*friendly_team.goalie());
        }

        auto best_enemy_shot_opt = Evaluation::calcBestShotOnFriendlyGoal(
            field, friendly_team, enemy_team, enemy_robot, ROBOT_MAX_RADIUS_METERS,
            robots_to_ignore);

        Vector enemy_shot_vector = Vector(0, 0);
        Point position_to_block_shot(enemy_robot.position());
        if (best_enemy_shot_opt)
        {
            enemy_shot_vector = best_enemy_shot_opt->first - enemy_robot.position();
            auto middle_shot_vector =
                    enemy_shot_vector.rotate(best_enemy_shot_opt->second / 2).norm();
            position_to_block_shot =
                    enemy_robot.position() + middle_shot_vector * shadow_distance;
        }
        else
        {
            enemy_shot_vector = field.friendlyGoal() - enemy_robot.position();
            position_to_block_shot =
                    enemy_robot.position() + enemy_shot_vector.norm() * shadow_distance;
        }

        // try to steal the ball and yeet it away if the enemy robot has already
        // received the pass
        if (Evaluation::robotHasPossession(ball, enemy_robot) && ball.velocity().len() <
            Util::DynamicParameters::DefenseShadowEnemyTactic::ball_steal_speed.value()) {
            yield(move_action.updateStateAndGetNextIntent(
                    *robot, ball.position(),
                    enemy_shot_vector.orientation() + Angle::half(), 0,
                    true, false, AutokickType::AUTOCHIP));
        } else {
            Angle facing_enemy_robot = (enemy_robot.position() - robot->position()).orientation();
            yield(move_action.updateStateAndGetNextIntent(
                    *robot, position_to_block_shot, facing_enemy_robot, 0,
                    false, false, AutokickType::AUTOCHIP));
        }

    } while (!move_action.done());
}
