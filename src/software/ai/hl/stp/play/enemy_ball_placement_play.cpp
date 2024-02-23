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

void EnemyBallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                            const World &world)
{
    /*
     * Set up 2 crease defenders and 3 robots to stay near the ball without interfering
     *
     *
     *                          placement point
     *                                +
     *
     *                     o    x  enemy robot with ball
     *      move robots      o
     *                         o
     *
     *   crease defenders  o   o
     *
     *        goalie         o
     *                    +-----+
     */

    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defenders = {
        // TODO-AKHIL: Remove this hard-coded value
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config())};

    std::array<std::shared_ptr<MoveTactic>, 3> move_tactics = {
        std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(),
    };

    std::optional<Point> raw_placement_point = world.gameState().getBallPlacementPoint();
    Point placement_point = raw_placement_point.has_value() ? raw_placement_point.value()
                                                            : world.ball().position();
    LOG(INFO) << "Placement point: " << placement_point;

    do
    {
        double distance_to_keep = ENEMY_BALL_PLACEMENT_DISTANCE_METERS + 0.2;

        PriorityTacticVector tactics_to_run = {{}};

        // Create crease defenders
        crease_defenders[0]->updateControlParams(
            placement_point, TbotsProto::CreaseDefenderAlignment::LEFT);
        crease_defenders[1]->updateControlParams(
            placement_point, TbotsProto::CreaseDefenderAlignment::RIGHT);

        tactics_to_run[0].emplace_back(crease_defenders[0]);
        tactics_to_run[0].emplace_back(crease_defenders[1]);

        // Create move tactics
        Vector positioning_vector = (world.ball().position() - placement_point);

        // If the ball is nearly placed, then adjust move tactics to position between
        // friendly goal and the ball
        if (positioning_vector.length() < 0.5)
        {
            positioning_vector =
                world.field().friendlyGoalCenter() - world.ball().position();
        }
        positioning_vector = positioning_vector.normalize() * distance_to_keep;

        Vector left_vector  = positioning_vector.rotate(Angle::fromDegrees(-30));
        Vector right_vector = positioning_vector.rotate(Angle::fromDegrees(30));

        Point center = world.ball().position() + positioning_vector;
        Point left   = world.ball().position() + left_vector;
        Point right  = world.ball().position() + right_vector;

        move_tactics[0]->updateControlParams(
            center, positioning_vector.orientation() + Angle::half(), 0);
        move_tactics[1]->updateControlParams(
            left, left_vector.orientation() + Angle::half(), 0);
        move_tactics[2]->updateControlParams(
            right, right_vector.orientation() + Angle::half(), 0);
        tactics_to_run[0].emplace_back(move_tactics[0]);
        tactics_to_run[0].emplace_back(move_tactics[1]);
        tactics_to_run[0].emplace_back(move_tactics[2]);

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);

    } while (true);
}

static TGenericFactory<std::string, Play, EnemyBallPlacementPlay, TbotsProto::AiConfig>
    factory;
