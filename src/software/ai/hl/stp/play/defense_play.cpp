#include "software/ai/hl/stp/play/defense_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

DefensePlay::DefensePlay(std::shared_ptr<const PlayConfig> config) : Play(config) {}

bool DefensePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::ENEMY);
}

bool DefensePlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::ENEMY);
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    bool enemy_team_can_pass =
        play_config->getEnemyCapabilityConfig()->getEnemyTeamCanPass()->value();

    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig());

    auto shoot_goal_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(5), std::nullopt, true,
        play_config->getShootGoalTacticConfig());

    auto defense_shadow_enemy_tactic = std::make_shared<DefenseShadowEnemyTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), true,
        3 * ROBOT_MAX_RADIUS_METERS, play_config->getDefenseShadowEnemyTacticConfig());

    std::shared_ptr<ShadowEnemyTactic> shadow_enemy_tactic =
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, world.ball(), 0.5,
                                            enemy_team_can_pass, false);


    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false)};

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        shoot_goal_tactic->updateControlParams(std::nullopt);

        PriorityTacticVector result = {{goalie_tactic, shoot_goal_tactic}};

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        // Assign ShadowEnemy tactics until we have every enemy covered. If there any
        // extra friendly robots, have them perform a reasonable default defensive tactic
        if (enemy_threats.size() > 0)
        {
            defense_shadow_enemy_tactic->updateControlParams(enemy_threats.at(1));
            result[0].emplace_back(defense_shadow_enemy_tactic);
        }
        else
        {
            auto swarm_ball_tactics = moveRobotsToSwarmEnemyWithBall(move_tactics, world);
            result[0].insert(result[0].end(), swarm_ball_tactics.begin(),
                             swarm_ball_tactics.end());
        }

        if (enemy_threats.size() > 1)
        {
            shadow_enemy_tactic->updateControlParams(enemy_threats.at(0),
                                                     ROBOT_MAX_RADIUS_METERS * 3, 0);
            result[0].emplace_back(shadow_enemy_tactic);
        }
        else
        {
            auto nearest_enemy_robot =
                world.enemyTeam().getNearestRobot(world.ball().position());
            if (nearest_enemy_robot)
            {
                Point block_point =
                    nearest_enemy_robot->position() +
                    Vector::createFromAngle(nearest_enemy_robot->orientation()) *
                        ROBOT_MAX_RADIUS_METERS * 3;
                move_tactics[1]->updateControlParams(
                    block_point, nearest_enemy_robot->orientation() + Angle::half(), 0.0);
                result[0].emplace_back(move_tactics[1]);
            }
            else
            {
                LOG(WARNING)
                    << "There are no enemy robots so a MoveTactic is not being assigned";
            }
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

std::vector<std::shared_ptr<MoveTactic>> DefensePlay::moveRobotsToSwarmEnemyWithBall(
    std::vector<std::shared_ptr<MoveTactic>> move_tactics, const World &world)
{
    auto nearest_enemy_robot = world.enemyTeam().getNearestRobot(world.ball().position());
    if (nearest_enemy_robot)
    {
        Point block_point = nearest_enemy_robot->position() +
                            Vector::createFromAngle(nearest_enemy_robot->orientation()) *
                                ROBOT_MAX_RADIUS_METERS * 3;
        move_tactics[0]->updateControlParams(
            block_point, nearest_enemy_robot->orientation() + Angle::half(), 0.0);
        move_tactics[1]->updateControlParams(
            block_point, nearest_enemy_robot->orientation() + Angle::half(), 0.0);
        return move_tactics;
    }
    else
    {
        // somehow they have 0 robots
        LOG(WARNING) << "0 enemy robots so we hit a very stupid fallback case";
        return {};
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, PlayConfig> factory;
