#include "software/ai/hl/stp/play/defense_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/hl/stp/tactic/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

DefensePlay::DefensePlay(std::shared_ptr<const PlayConfig> config)
      : play_config(config)
{
}

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
        DynamicParameters->getEnemyCapabilityConfig()->getEnemyTeamCanPass()->value();

    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    auto shoot_goal_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(5), std::nullopt, true);

    auto defense_shadow_enemy_tactic = std::make_shared<DefenseShadowEnemyTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), true,
        3 * ROBOT_MAX_RADIUS_METERS);

    std::shared_ptr<ShadowEnemyTactic> shadow_enemy_tactic =
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, world.ball(), 0.5,
                                            enemy_team_can_pass, false);


    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false)};

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        // If we have any crease defenders, we don't want the goalie tactic to consider
        // them when deciding where to block
        Team friendly_team_for_goalie = world.friendlyTeam();

        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            if (crease_defender_tactic->getAssignedRobot())
            {
                friendly_team_for_goalie.removeRobotWithId(
                    crease_defender_tactic->getAssignedRobot()->id());
            }
        }
        shoot_goal_tactic->updateControlParams(std::nullopt);

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic, shoot_goal_tactic};

        // Update crease defenders
        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            result.emplace_back(crease_defender_tactic);
        }

        // Assign ShadowEnemy tactics until we have every enemy covered. If there any
        // extra friendly robots, have them perform a reasonable default defensive tactic
        if (enemy_threats.size() > 0)
        {
            defense_shadow_enemy_tactic->updateControlParams(enemy_threats.at(1));
            result.emplace_back(defense_shadow_enemy_tactic);
        }
        else
        {
            auto swarm_ball_tactics = moveRobotsToSwarmEnemyWithBall(move_tactics, world);
            result.insert(result.end(), swarm_ball_tactics.begin(),
                          swarm_ball_tactics.end());
        }

        if (enemy_threats.size() > 1)
        {
            shadow_enemy_tactic->updateControlParams(enemy_threats.at(0),
                                                     ROBOT_MAX_RADIUS_METERS * 3);
            result.emplace_back(shadow_enemy_tactic);
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
                result.emplace_back(move_tactics[1]);
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
