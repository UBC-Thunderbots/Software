#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_free_kicker_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreekickPlay::EnemyFreekickPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config)
{
}

bool EnemyFreekickPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirFreeKick();
}

bool EnemyFreekickPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirFreeKick();
}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // Init our goalie tactic
    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig());

    // Init a Crease Defender Tactic
    auto crease_defender_tactic = std::make_shared<CreaseDefenderTactic>(
        play_config->getRobotNavigationObstacleConfig());

    // Init FreeKickShadower tactics (these robots will both block the enemy robot taking
    // a free kick (at most we will have 2
    auto shadow_free_kicker_1 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::LEFT, world.enemyTeam(), world.ball(), world.field(),
        true);
    auto shadow_free_kicker_2 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::RIGHT, world.enemyTeam(), world.ball(), world.field(),
        true);

    // Init Shadow Enemy Tactics for extra robots
    auto shadow_tactic_main = std::make_shared<ShadowEnemyTactic>();
    auto shadow_tactic_secondary = std::make_shared<ShadowEnemyTactic>();

    // Init Move Tactics for extra robots (These will be used if there are no robots to
    // shadow)
    auto move_tactic_main      = std::make_shared<MoveTactic>(true);
    auto move_tactic_secondary = std::make_shared<MoveTactic>(true);

    do
    {
        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{goalie_tactic}};

        // Get all enemy threats
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        // Add Freekick shadower tactics
        tactics_to_run[0].emplace_back(shadow_free_kicker_1);
        tactics_to_run[0].emplace_back(shadow_free_kicker_2);
        // Add Crease defender tactic
        crease_defender_tactic->updateControlParams(world.ball().position(),
                                                    CreaseDefenderAlignment::CENTRE);
        tactics_to_run[0].emplace_back(crease_defender_tactic);


        // Assign ShadowEnemy tactics until we have every enemy covered. If there are not
        // enough threats to shadow, move our robots to block the friendly net
        if (enemy_threats.size() == 0)
        {
            move_tactic_main->updateControlParams(
                world.field().friendlyGoalCenter() +
                    Vector(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoalCenter())
                    .orientation(),
                0);
            move_tactic_main->updateControlParams(
                world.field().friendlyGoalCenter() +
                    Vector(0, -2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoalCenter())
                    .orientation(),
                0);

            tactics_to_run[0].emplace_back(move_tactic_main);
            tactics_to_run[0].emplace_back(move_tactic_secondary);
        }
        if (enemy_threats.size() == 1)
        {
            shadow_tactic_main->updateControlParams(enemy_threats.at(1),
                                                    ROBOT_MAX_RADIUS_METERS * 3);
            move_tactic_main->updateControlParams(
                world.field().friendlyGoalCenter() +
                    Vector(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoalCenter())
                    .orientation(),
                0);

            tactics_to_run[0].emplace_back(shadow_tactic_main);
            tactics_to_run[0].emplace_back(move_tactic_main);
        }
        if (enemy_threats.size() >= 2)
        {
            shadow_tactic_main->updateControlParams(enemy_threats.at(1),
                                                    ROBOT_MAX_RADIUS_METERS * 3);
            shadow_tactic_secondary->updateControlParams(enemy_threats.at(2),
                                                         ROBOT_MAX_RADIUS_METERS * 3);

            tactics_to_run[0].emplace_back(shadow_tactic_main);
            tactics_to_run[0].emplace_back(shadow_tactic_secondary);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, PlayConfig> factory;
