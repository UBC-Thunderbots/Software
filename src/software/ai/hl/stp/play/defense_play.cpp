#include "software/ai/hl/stp/play/defense_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

DefensePlay::DefensePlay(std::shared_ptr<const PlayConfig> config) : Play(config, true) {}

bool DefensePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           world.getTeamWithPossession() == TeamSide::ENEMY &&
           world.getTeamWithPossessionConfidence() >= 1.0;
}

bool DefensePlay::invariantHolds(const World &world) const
{
    LOG(INFO) << "DEFENSE PLAY CONFIDENCE: " << world.getTeamWithPossessionConfidence();
    return world.gameState().isPlaying() &&
           world.getTeamWithPossession() == TeamSide::ENEMY &&
           world.getTeamWithPossessionConfidence() >= 1.0;
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto attacker_tactic =
        std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());

    std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>(),
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>(),
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>(),
    };

    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false)};

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        PriorityTacticVector result = {{}};

        if (attacker_tactic->done())
        {
            attacker_tactic =
                std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());
        }
        result[0].emplace_back(attacker_tactic);

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        // Determine how many "immediate" enemy threats there are. If there is only one we
        // have both shadow enemy tactics swarm and block the "immediate" threat.
        // Otherwise we assign ShadowEnemy tactics for the next highest threats. If there
        // any extra friendly robots, have them perform a reasonable default defensive
        // tactic
        int immediate_enemy_threats = static_cast<int>(std::count_if(
            enemy_threats.begin(), enemy_threats.end(), [this, world](auto enemy_threat) {
                return distance(world.field().friendlyGoal(),
                                enemy_threat.robot.position()) <
                       play_config->getDefensePlayConfig()
                           ->getImmediateThreatDistance()
                           ->value();
            }));


        if (immediate_enemy_threats == 1)
        {
            shadow_enemy_tactics[0]->updateControlParams(enemy_threats.at(0),
                                                         ROBOT_SHADOWING_DISTANCE_METERS);
            shadow_enemy_tactics[1]->updateControlParams(enemy_threats.at(0),
                                                         ROBOT_SHADOWING_DISTANCE_METERS);
            result[0].insert(result[0].end(), shadow_enemy_tactics.begin(),
                             shadow_enemy_tactics.end());
            std::get<2>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                                      CreaseDefenderAlignment::CENTRE);
            result[0].emplace_back(std::get<2>(crease_defender_tactics));
        }
        else if (enemy_threats.size() > 0)
        {
            shadow_enemy_tactics[0]->updateControlParams(enemy_threats.at(0),
                                                         ROBOT_SHADOWING_DISTANCE_METERS);
            result[0].emplace_back(shadow_enemy_tactics[0]);
        }

        if (enemy_threats.size() > 1)
        {
            shadow_enemy_tactics[1]->updateControlParams(enemy_threats.at(1),
                                                         ROBOT_SHADOWING_DISTANCE_METERS);
            result[0].emplace_back(shadow_enemy_tactics[1]);
            std::get<2>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                                      CreaseDefenderAlignment::CENTRE);
            result[0].emplace_back(std::get<2>(crease_defender_tactics));
            for (unsigned int i = 2;
                 i < std::min(enemy_threats.size(), shadow_enemy_tactics.size()); i++)
            {
                shadow_enemy_tactics[i]->updateControlParams(
                    enemy_threats.at(i), ROBOT_SHADOWING_DISTANCE_METERS);
                result[0].emplace_back(shadow_enemy_tactics[i]);
            }
        }
        auto nearest_enemy_robot =
            world.enemyTeam().getNearestRobot(world.ball().position());
        if (nearest_enemy_robot)
        {
            // Blocks in front of where the closest enemy robot is
            Point block_point =
                nearest_enemy_robot->position() +
                Vector::createFromAngle(nearest_enemy_robot->orientation()) *
                    ROBOT_SHADOWING_DISTANCE_METERS;
            move_tactics[0]->updateControlParams(
                block_point, nearest_enemy_robot->orientation() + Angle::half(), 0.0);
            result[0].emplace_back(move_tactics[0]);
        }
        else
        {
            LOG(WARNING)
                << "There are no enemy robots so a MoveTactic is not being assigned";
        }

        for (unsigned int i = 1; i < move_tactics.size(); i++)
        {
            move_tactics[i]->updateControlParams(
                world.field().friendlyDefenseArea().posXNegYCorner() + Vector(1, 0) +
                    (world.field().friendlyDefenseArea().posXPosYCorner() -
                     world.field().friendlyDefenseArea().posXNegYCorner()) *
                        (i - 1) / 3.0,
                Angle::zero(), 0.0);
            result[0].emplace_back(move_tactics[i]);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, PlayConfig> factory;
