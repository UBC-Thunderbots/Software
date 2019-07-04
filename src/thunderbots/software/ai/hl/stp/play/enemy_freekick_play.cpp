#include "ai/hl/stp/play/enemy_freekick_play.h"

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/block_shot_path_tactic.h"
#include "ai/hl/stp/tactic/crease_defender_tactic.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/shadow_freekicker.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "ai/world/game_state.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"


const std::string EnemyFreekickPlay::name = "Enemy Freekick Play";

std::string EnemyFreekickPlay::getName() const
{
    return EnemyFreekickPlay::name;
}

bool EnemyFreekickPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirFreeKick();
}

bool EnemyFreekickPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirFreeKick() && !world.gameState().isPlaying();
}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Init our goalie tactic
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    // Init a Crease Defender Tactic
    auto crease_defender_tactic = std::make_shared<CreaseDefenderTactic>(
        world.field(), world.ball(), world.friendlyTeam(), world.enemyTeam(),
        CreaseDefenderTactic::LeftOrRight::RIGHT);

    // Init FreeKickShadower tactics (these robots will both block the enemy robot taking
    // a free kick (at most we will have 2
    auto shadow_freekicker_1 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::First, world.enemyTeam(), world.ball(), world.field(),
        true);
    auto shadow_freekicker_2 = std::make_shared<ShadowFreekickerTactic>(
        ShadowFreekickerTactic::Second, world.enemyTeam(), world.ball(), world.field(),
        true);

    // Init Shadow Enemy Tactics for extra robots
    auto shadow_tactic_main = std::make_shared<ShadowEnemyTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), true, true);
    auto shadow_tactic_secondary = std::make_shared<ShadowEnemyTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), true, true);

    // Init Move Tactics for extra robots (These will be used if there are no robots to
    // shadow)
    auto move_tactic_main      = std::make_shared<MoveTactic>(true);
    auto move_tactic_secondary = std::make_shared<MoveTactic>(true);

    do
    {
        // Create tactic vector (starting with Goalie)
        std::vector<std::shared_ptr<Tactic>> tactics_to_run = {goalie_tactic};

        // Get all enemy threats
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

        // Check if the enemy is passing-capable
        bool enemy_team_can_pass =
            Util::DynamicParameters::EnemyCapability::enemy_team_can_pass.value();

        // Update goalie tactic
        goalie_tactic->updateParams(world.ball(), world.field(), world.friendlyTeam(),
                                    world.enemyTeam());

        // Update free kicke shadowers
        shadow_freekicker_1->updateParams(world.enemyTeam(), world.ball());
        shadow_freekicker_2->updateParams(world.enemyTeam(), world.ball());

        // Update crease defenders
        crease_defender_tactic->updateParams(world.ball(), world.field(),
                                             world.friendlyTeam(), world.enemyTeam());

        // Add Freekick shadower tactics
        tactics_to_run.emplace_back(shadow_freekicker_1);
        tactics_to_run.emplace_back(shadow_freekicker_2);
        // Add Crease defender tactic
        tactics_to_run.emplace_back(crease_defender_tactic);


        // Assign ShadowEnemy tactics until we have every enemy covered. If there are not
        // enough threats to shadow, move our robots to block the friendly net
        if (enemy_threats.size() == 0)
        {
            move_tactic_main->updateParams(
                world.field().friendlyGoal() + Point(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoal()).orientation(),
                0);
            move_tactic_main->updateParams(
                world.field().friendlyGoal() + Point(0, -2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoal()).orientation(),
                0);

            tactics_to_run.emplace_back(move_tactic_main);
            tactics_to_run.emplace_back(move_tactic_secondary);
        }
        if (enemy_threats.size() == 1)
        {
            shadow_tactic_main->updateParams(
                enemy_threats.at(1), world.field(), world.friendlyTeam(),
                world.enemyTeam(), ROBOT_MAX_RADIUS_METERS * 3, enemy_team_can_pass);
            move_tactic_main->updateParams(
                world.field().friendlyGoal() + Point(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world.ball().position() - world.field().friendlyGoal()).orientation(),
                0);

            tactics_to_run.emplace_back(shadow_tactic_main);
            tactics_to_run.emplace_back(move_tactic_main);
        }
        if (enemy_threats.size() >= 2)
        {
            shadow_tactic_main->updateParams(
                enemy_threats.at(1), world.field(), world.friendlyTeam(),
                world.enemyTeam(), ROBOT_MAX_RADIUS_METERS * 3, enemy_team_can_pass);
            shadow_tactic_secondary->updateParams(
                enemy_threats.at(2), world.field(), world.friendlyTeam(),
                world.enemyTeam(), ROBOT_MAX_RADIUS_METERS * 3, enemy_team_can_pass);

            tactics_to_run.emplace_back(shadow_tactic_main);
            tactics_to_run.emplace_back(shadow_tactic_secondary);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<EnemyFreekickPlay> factory;
