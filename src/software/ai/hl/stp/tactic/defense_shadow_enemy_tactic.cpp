#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/util/logger/init.h"
#include "software/util/parameter/dynamic_parameters.h"

DefenseShadowEnemyTactic::DefenseShadowEnemyTactic(
    const Field &field, const Team &friendly_team, const Team &enemy_team,
    const Ball &ball, bool ignore_goalie, double shadow_distance, bool loop_forever)
    : Tactic(loop_forever),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ball(ball),
      ignore_goalie(ignore_goalie),
      shadow_distance(shadow_distance)
{
}

std::string DefenseShadowEnemyTactic::getName() const
{
    return "Defense Shadow Enemy Tactic";
}

void DefenseShadowEnemyTactic::updateWorldParams(const Field &field,
                                                 const Team &friendly_team,
                                                 const Team &enemy_team, const Ball &ball)
{
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->ball          = ball;
}

void DefenseShadowEnemyTactic::updateControlParams(
    const Evaluation::EnemyThreat &enemy_threat)
{
    this->enemy_threat = enemy_threat;
}

double DefenseShadowEnemyTactic::calculateRobotCost(const Robot &robot,
                                                    const World &world)
{
    if (!enemy_threat)
    {
        return 0;
    }
    // Prefer robots closer to the enemy being shadowed
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - enemy_threat->robot.position()).len() /
                  world.field().totalXLength();
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
            LOG(WARNING) << "Running DefenseShadowEnemyTactic without an enemy threat";
            yield(stop_action.updateStateAndGetNextIntent(*robot, false));
        }

        Robot enemy_robot                   = enemy_threat->robot;
        std::vector<Robot> robots_to_ignore = {*robot};

        if (ignore_goalie && friendly_team.goalie())
        {
            robots_to_ignore.emplace_back(*friendly_team.goalie());
        }

        auto best_enemy_shot_opt = Evaluation::calcBestShotOnFriendlyGoal(
            field, friendly_team, enemy_team, enemy_robot, ROBOT_MAX_RADIUS_METERS,
            robots_to_ignore);

        Vector enemy_shot_vector = field.friendlyGoal() - enemy_robot.position();
        Point position_to_block_shot =
            enemy_robot.position() + enemy_shot_vector.norm(shadow_distance);
        if (best_enemy_shot_opt)
        {
            enemy_shot_vector =
                best_enemy_shot_opt->getPointToShootAt() - enemy_robot.position();
            position_to_block_shot =
                enemy_robot.position() + enemy_shot_vector.norm(shadow_distance);
        }

        // try to steal the ball and yeet it away if the enemy robot has already
        // received the pass
        if (*Evaluation::robotHasPossession(ball, enemy_robot) &&
            ball.velocity().len() <
                Util::DynamicParameters->getDefenseShadowEnemyTacticConfig()
                    ->BallStealSpeed()
                    ->value())
        {
            yield(move_action.updateStateAndGetNextIntent(
                *robot, ball.position(), enemy_shot_vector.orientation() + Angle::half(),
                0, DribblerEnable::ON, MoveType::NORMAL, AutokickType::AUTOCHIP));
        }
        else
        {
            Angle facing_enemy_robot =
                (enemy_robot.position() - robot->position()).orientation();
            yield(move_action.updateStateAndGetNextIntent(
                *robot, position_to_block_shot, facing_enemy_robot, 0,
                DribblerEnable::OFF, MoveType::NORMAL, AutokickType::AUTOCHIP));
        }

    } while (!move_action.done());
}

void DefenseShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
