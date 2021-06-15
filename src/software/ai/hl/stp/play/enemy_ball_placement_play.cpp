#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"

EnemyBallPlacementPlay::EnemyBallPlacementPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
{
}

bool EnemyBallPlacementPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirBallPlacement();
}

bool EnemyBallPlacementPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirBallPlacement();
}

void EnemyBallPlacementPlay::ballPlacementNoShadow(
    TacticCoroutine::push_type &yield, const World &world,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders,
    std::array<std::shared_ptr<MoveTactic>, 2> move_tactics, Point placement_point)
{
    /*
     * Set up 3 crease defenders to sit by crease, and put two robots behind the ball
     * placement point
     *
     *               placement point
     *                       +
     *
     *      move robots    o   o
     *
     *
     * crease defenders  o   o   o
     *
     *        goalie         o
     *                    +-----+
     */

    do
    {
        PriorityTacticVector tactics_to_run = {{}};

        crease_defenders[0]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::LEFT);
        crease_defenders[1]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::RIGHT);
        crease_defenders[2]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::CENTRE);

        tactics_to_run[0].emplace_back(crease_defenders[0]);
        tactics_to_run[0].emplace_back(crease_defenders[1]);
        tactics_to_run[0].emplace_back(crease_defenders[2]);


        move_tactics[0]->updateControlParams(
            world.gameState().getBallPlacementPoint().value() +
                Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
            Angle::zero(), 0);
        move_tactics[1]->updateControlParams(
            world.gameState().getBallPlacementPoint().value() +
                Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS),
            Angle::zero(), 0);

        tactics_to_run[0].emplace_back(move_tactics[0]);
        tactics_to_run[0].emplace_back(move_tactics[1]);
        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

void EnemyBallPlacementPlay::ballPlacementWithShadow(
    TacticCoroutine::push_type &yield, const World &world,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders,
    std::array<std::shared_ptr<MoveTactic>, 2> move_tactics, Point placement_point)
{
    /*
     * Set up 3 crease defenders to sit by crease, and put two robots behind the ball
     * placement point
     *
     *
     *               placement point
     *                       +
     *
     *     Shadow enemy    o  x  enemy robot
     *
     *      move robots      o
     *
     * crease defenders  o   o   o
     *
     *        goalie         o
     *                    +-----+
     */
    auto shadow_enemy = std::make_shared<ShadowEnemyTactic>();

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{}};

        crease_defenders[0]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::LEFT);
        crease_defenders[1]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::RIGHT);
        crease_defenders[2]->updateControlParams(placement_point,
                                                 CreaseDefenderAlignment::CENTRE);

        tactics_to_run[0].emplace_back(crease_defenders[0]);
        tactics_to_run[0].emplace_back(crease_defenders[1]);
        tactics_to_run[0].emplace_back(crease_defenders[2]);

        // if no threats, send two robots near placement point
        if (enemy_threats.size() == 0)
        {
            move_tactics[0]->updateControlParams(
                placement_point +
                    Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
                Angle::zero(), 0);
            move_tactics[1]->updateControlParams(
                placement_point +
                    Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS),
                Angle::zero(), 0);
            tactics_to_run[0].emplace_back(move_tactics[0]);
            tactics_to_run[0].emplace_back(move_tactics[1]);
        }

        // if there are threats, send one robot to placement point, and one shadows
        else
        {
            move_tactics[0]->updateControlParams(
                placement_point + Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, 0),
                Angle::zero(), 0);
            shadow_enemy->updateControlParams(enemy_threats.at(1),
                                              ROBOT_MAX_RADIUS_METERS * 3);

            tactics_to_run[0].emplace_back(move_tactics[0]);
            tactics_to_run[0].emplace_back(shadow_enemy);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

void EnemyBallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                            const World &world)
{
    std::optional<Point> placement_point = world.gameState().getBallPlacementPoint();

    std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    std::array<std::shared_ptr<MoveTactic>, 2> move_tactics = {
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
    };

    if (placement_point.has_value())
    {
        if (placement_point.value().x() > 0)
        {
            ballPlacementWithShadow(yield, world, crease_defenders, move_tactics,
                                    placement_point.value());
        }
        else
        {
            ballPlacementNoShadow(yield, world, crease_defenders, move_tactics,
                                  placement_point.value());
        }
    }
    // if there is no placement_point, set up at middle of field with two robots
    else
    {
        Point default_point =
            Point(world.field().centerPoint().x() - 2, world.field().centerPoint().y());
        ballPlacementNoShadow(yield, world, crease_defenders, move_tactics,
                              default_point);
    }
}

static TGenericFactory<std::string, Play, EnemyBallPlacementPlay, PlayConfig> factory;
