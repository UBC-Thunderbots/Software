#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/mutable_tactic_visitor.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"

DefenseShadowEnemyTactic::DefenseShadowEnemyTactic(const Field &field,
                                                   const Team &friendly_team,
                                                   const Team &enemy_team,
                                                   const Ball &ball, bool ignore_goalie,
                                                   double shadow_distance)
    : Tactic(true),
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
    double cost = (robot.position() - enemy_threat->robot.position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void DefenseShadowEnemyTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action = std::make_shared<MoveAction>(false);
    auto stop_action = std::make_shared<StopAction>(true);

    do
    {
        if (!enemy_threat)
        {
            LOG(WARNING) << "Running DefenseShadowEnemyTactic without an enemy threat";
            stop_action->updateControlParams(*robot, false);
            yield(stop_action);
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
            enemy_robot.position() + enemy_shot_vector.normalize(shadow_distance);
        if (best_enemy_shot_opt)
        {
            enemy_shot_vector =
                best_enemy_shot_opt->getPointToShootAt() - enemy_robot.position();
            position_to_block_shot =
                enemy_robot.position() + enemy_shot_vector.normalize(shadow_distance);
        }

        // try to steal the ball and yeet it away if the enemy robot has already
        // received the pass
        if (*Evaluation::robotHasPossession(ball, enemy_robot) &&
            ball.velocity().length() < Util::DynamicParameters->getAIConfig()
                                           ->getDefenseShadowEnemyTacticConfig()
                                           ->BallStealSpeed()
                                           ->value())
        {
            move_action->updateControlParams(
                *robot, ball.position(), enemy_shot_vector.orientation() + Angle::half(),
                0, DribblerEnable::ON, MoveType::NORMAL, AutokickType::AUTOCHIP,
                BallCollisionType::AVOID);
            yield(move_action);
        }
        else
        {
            Angle facing_enemy_robot =
                (enemy_robot.position() - robot->position()).orientation();
            move_action->updateControlParams(*robot, position_to_block_shot,
                                             facing_enemy_robot, 0, DribblerEnable::OFF,
                                             MoveType::NORMAL, AutokickType::AUTOCHIP,
                                             BallCollisionType::AVOID);
            yield(move_action);
        }

    } while (!move_action->done());
}

void DefenseShadowEnemyTactic::accept(MutableTacticVisitor &visitor)
{
    visitor.visit(*this);
}

Ball DefenseShadowEnemyTactic::getBall() const
{
    return this->ball;
}

Field DefenseShadowEnemyTactic::getField() const
{
    return this->field;
}

Team DefenseShadowEnemyTactic::getFriendlyTeam() const
{
    return this->friendly_team;
}

Team DefenseShadowEnemyTactic::getEnemyTeam() const
{
    return this->enemy_team;
}
