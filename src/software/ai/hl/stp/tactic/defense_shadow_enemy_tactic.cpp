#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/autochip_move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/logger/logger.h"

DefenseShadowEnemyTactic::DefenseShadowEnemyTactic(
    const Field &field, const Team &friendly_team, const Team &enemy_team,
    const Ball &ball, bool ignore_goalie, double shadow_distance,
    std::shared_ptr<const DefenseShadowEnemyTacticConfig>
        defense_shadow_enemy_tactic_config)
    : Tactic(true, {RobotCapability::Move}),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      ball(ball),
      ignore_goalie(ignore_goalie),
      shadow_distance(shadow_distance),
      defense_shadow_enemy_tactic_config(defense_shadow_enemy_tactic_config)
{
}

void DefenseShadowEnemyTactic::updateWorldParams(const World &world)
{
    this->field         = world.field();
    this->friendly_team = world.friendlyTeam();
    this->enemy_team    = world.enemyTeam();
    this->ball          = world.ball();
}


void DefenseShadowEnemyTactic::updateControlParams(const EnemyThreat &enemy_threat)
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
    auto autochip_move_action = std::make_shared<AutochipMoveAction>(false);
    auto stop_action          = std::make_shared<StopAction>(true);

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

        auto best_enemy_shot_opt =
            calcBestShotOnGoal(field, friendly_team, enemy_team, enemy_robot.position(),
                               TeamType::FRIENDLY, robots_to_ignore);

        Vector enemy_shot_vector = field.friendlyGoalCenter() - enemy_robot.position();
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
        if (enemy_robot.isNearDribbler(ball.position()) &&
            ball.velocity().length() <
                defense_shadow_enemy_tactic_config->BallStealSpeed()->value())
        {
            autochip_move_action->updateControlParams(
                *robot, ball.position(), enemy_shot_vector.orientation() + Angle::half(),
                0, DribblerMode::MAX_FORCE, YEET_CHIP_DISTANCE_METERS,
                BallCollisionType::AVOID);
            yield(autochip_move_action);
        }
        else
        {
            Angle facing_enemy_robot =
                (enemy_robot.position() - robot->position()).orientation();
            autochip_move_action->updateControlParams(
                *robot, position_to_block_shot, facing_enemy_robot, 0, DribblerMode::OFF,
                YEET_CHIP_DISTANCE_METERS, BallCollisionType::AVOID);
            yield(autochip_move_action);
        }

    } while (!autochip_move_action->done());
}

void DefenseShadowEnemyTactic::accept(TacticVisitor &visitor) const
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
