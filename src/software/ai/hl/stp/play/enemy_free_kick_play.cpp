#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_free_kicker_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"
static const double ROBOT_SHADOWING_DISTANCE_METERS = ROBOT_MAX_RADIUS_METERS * 3;

EnemyFreekickPlay::EnemyFreekickPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
{
}

bool EnemyFreekickPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirFreeKick();
}

bool EnemyFreekickPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirFreeKick() &&
                    world.ball().velocity().length() < 1.0;
}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    auto crease_defender_tactic_1 = std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig());
    auto crease_defender_tactic_2 = std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig());

    // Init FreeKickShadower tactics (these robots will both block the enemy robot taking
    // a free kick, at most we will have 2)
    auto shadow_free_kicker_1 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::LEFT, world.enemyTeam(), world.ball(), world.field(),
        true);
    auto shadow_free_kicker_2 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::RIGHT, world.enemyTeam(), world.ball(), world.field(),
        true);

    // Init Shadow Enemy Tactics for extra robots
    auto shadow_1 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_2 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_3 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_4 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_5 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_6 = std::make_shared<ShadowEnemyTactic>();

    do
    {
        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{}};

        // Get all enemy threats
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        tactics_to_run[0].emplace_back(shadow_free_kicker_1);
        tactics_to_run[0].emplace_back(shadow_free_kicker_2);

        crease_defender_tactic_1->updateControlParams(
                world.ball().position(), CreaseDefenderAlignment::LEFT);
        crease_defender_tactic_2->updateControlParams(
                world.ball().position(), CreaseDefenderAlignment::RIGHT);

        tactics_to_run[0].emplace_back(crease_defender_tactic_1);
        tactics_to_run[0].emplace_back(crease_defender_tactic_2);

            // LOL can this be jankier
            if (enemy_threats.size() > 1)
            {
                shadow_1
                    ->updateControlParams(enemy_threats.at(0),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            // LOL can this be jankier
            if (enemy_threats.size() > 1)
            {
                shadow_2
                    ->updateControlParams(enemy_threats.at(1),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            // LOL can this be jankier
            if (enemy_threats.size() > 2)
            {
                shadow_3
                    ->updateControlParams(enemy_threats.at(2),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            // LOL can this be jankier
            if (enemy_threats.size() > 3)
            {
                shadow_4
                    ->updateControlParams(enemy_threats.at(3),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            // LOL can this be jankier
            if (enemy_threats.size() > 4)
            {
                shadow_5
                    ->updateControlParams(enemy_threats.at(4),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            // LOL can this be jankier
            if (enemy_threats.size() > 5)
            {
                shadow_6
                    ->updateControlParams(enemy_threats.at(5),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            tactics_to_run[0].emplace_back(shadow_1);
            tactics_to_run[0].emplace_back(shadow_2);
            tactics_to_run[0].emplace_back(shadow_3);
            tactics_to_run[0].emplace_back(shadow_4);
            tactics_to_run[0].emplace_back(shadow_5);
            tactics_to_run[0].emplace_back(shadow_6);

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, PlayConfig> factory;
