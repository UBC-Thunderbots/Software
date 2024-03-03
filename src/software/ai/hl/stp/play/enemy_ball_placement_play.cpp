#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyBallPlacementPlay::EnemyBallPlacementPlay(TbotsProto::AiConfig config)
    : Play(config, true)
{
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

    bool enemy_at_ball = false;

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{}};

        crease_defenders[0]->updateControlParams(
            placement_point, TbotsProto::CreaseDefenderAlignment::LEFT);
        crease_defenders[1]->updateControlParams(
            placement_point, TbotsProto::CreaseDefenderAlignment::RIGHT);
        crease_defenders[2]->updateControlParams(
            placement_point, TbotsProto::CreaseDefenderAlignment::CENTRE);

        tactics_to_run[0].emplace_back(crease_defenders[0]);
        tactics_to_run[0].emplace_back(crease_defenders[1]);
        tactics_to_run[0].emplace_back(crease_defenders[2]);

        Vector ball_to_net =
            (world.ball().position() - world.field().friendlyGoalCenter())
                .normalize(-0.75 - ROBOT_MAX_RADIUS_METERS);

        Vector placement_to_net = (placement_point - world.field().friendlyGoalCenter())
                                      .normalize(-0.75 - ROBOT_MAX_RADIUS_METERS);

        // Check to see if the enemy has the ball. Once they do, we change our shadowing
        // behaviour
        for (const auto &enemy_robot : world.enemyTeam().getAllRobotsExceptGoalie())
        {
            if ((enemy_robot.position() - world.ball().position()).length() < 0.25)
            {
                enemy_at_ball = true;
            }
        }

        // If the enemy hasn't reached the ball yet, we use this flag to avoid shadowing
        // so that we don't interfere with the enemy robots going to pick up the ball
        if (!enemy_at_ball)
        {
            move_tactics[0]->updateControlParams(
                world.ball().position() + ball_to_net +
                    ball_to_net.perpendicular().normalize(1.25 * ROBOT_MAX_RADIUS_METERS),
                ball_to_net.orientation() + Angle::half(), 0,
                TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            move_tactics[1]->updateControlParams(
                world.ball().position() + ball_to_net -
                    ball_to_net.perpendicular().normalize(1.25 * ROBOT_MAX_RADIUS_METERS),
                ball_to_net.orientation() + Angle::half(), 0,
                TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            tactics_to_run[0].emplace_back(move_tactics[0]);
            tactics_to_run[0].emplace_back(move_tactics[1]);
        }
        // if no threats, send two robots near placement point
        else if (enemy_threats.size() == 0)
        {
            move_tactics[0]->updateControlParams(
                placement_point + placement_to_net +
                    placement_to_net.perpendicular().normalize(1.25 *
                                                               ROBOT_MAX_RADIUS_METERS),
                placement_to_net.orientation() + Angle::half(), 0,
                TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            move_tactics[1]->updateControlParams(
                placement_point + placement_to_net -
                    placement_to_net.perpendicular().normalize(1.25 *
                                                               ROBOT_MAX_RADIUS_METERS),
                placement_to_net.orientation() + Angle::half(), 0,
                TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            tactics_to_run[0].emplace_back(move_tactics[0]);
            tactics_to_run[0].emplace_back(move_tactics[1]);
        }
        // if there are threats, send one robot to placement point, and one shadows
        else
        {
            move_tactics[0]->updateControlParams(
                placement_point + placement_to_net,
                placement_to_net.orientation() + Angle::half(), 0,
                TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            // We need a big shadow distance to avoid "bullying" the robot ball placing.
            // Otherwise, we get a penalty
            shadow_enemy->updateControlParams(
                enemy_threats.at(0),
                ROBOT_MAX_RADIUS_METERS * 4);  // Leave 2 robot widths distance (~36cm)

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
        // TODO-AKHIL: Remove this hard-coded value
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config())};

    std::array<std::shared_ptr<MoveTactic>, 2> move_tactics = {
        std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(),
    };

    if (placement_point.has_value())
    {
        ballPlacementWithShadow(yield, world, crease_defenders, move_tactics,
                                placement_point.value());
    }
    // if there is no placement_point, use the ball position
    else
    {
        ballPlacementWithShadow(yield, world, crease_defenders, move_tactics,
                                world.ball().position());
    }
}

static TGenericFactory<std::string, Play, EnemyBallPlacementPlay, TbotsProto::AiConfig>
    factory;
