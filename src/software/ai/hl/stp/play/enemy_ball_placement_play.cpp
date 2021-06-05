#include "software/ai/hl/stp/play/enemy_ball_placement_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
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

void EnemyBallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world) {
    /*
     * Set up 3 crease defenders to sit by crease, and put two robots behind the ball placement point
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
    auto crease_defender_left = std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig());
    auto crease_defender_right = std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig());
    auto crease_defender_center = std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig());

    auto move_left= std::make_shared<MoveTactic>(true);
    auto move_right = std::make_shared<MoveTactic>(true);

    do
    {
        std::optional<Point> ball_point = world.gameState().getBallPlacementPoint();

        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{}};

        if(ball_point.has_value())
        {
            crease_defender_left->updateControlParams(ball_point.value(),
                                                    CreaseDefenderAlignment::LEFT);
            crease_defender_right->updateControlParams(ball_point.value(),
                                                    CreaseDefenderAlignment::RIGHT);
            crease_defender_center->updateControlParams(ball_point.value(),
                                                        CreaseDefenderAlignment::CENTRE);

            tactics_to_run[0].emplace_back(crease_defender_left);
            tactics_to_run[0].emplace_back(crease_defender_right);
            tactics_to_run[0].emplace_back(crease_defender_center);

            move_left->updateControlParams(
                    world.gameState().getBallPlacementPoint().value() + Vector(-0.75-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
                    Angle::zero(),
                    0);
			move_right->updateControlParams(
					world.gameState().getBallPlacementPoint().value() + Vector(-0.75-ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS),
                    Angle::zero(),
                    0);

			tactics_to_run[0].emplace_back(move_left);
			tactics_to_run[0].emplace_back(move_right);
        }
		else
		{
			crease_defender_left->updateControlParams(world.field().centerPoint(),
                                                    CreaseDefenderAlignment::LEFT);
            crease_defender_right->updateControlParams(world.field().centerPoint(),
                                                    CreaseDefenderAlignment::RIGHT);
            crease_defender_center->updateControlParams(world.field().centerPoint(),
                                                        CreaseDefenderAlignment::CENTRE);

            tactics_to_run[0].emplace_back(crease_defender_left);
            tactics_to_run[0].emplace_back(crease_defender_right);
            tactics_to_run[0].emplace_back(crease_defender_center);

            move_left->updateControlParams(
                    world.field().centerPoint() + Vector(-0.75 - ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
                    Angle::zero(),
                    0);
			move_right->updateControlParams(
					world.field().centerPoint() + Vector(0.75 + ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
                    Angle::zero(),
                    0);

			tactics_to_run[0].emplace_back(move_left);
			tactics_to_run[0].emplace_back(move_right);
		}

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);

}

static TGenericFactory<std::string, Play, EnemyBallPlacementPlay, PlayConfig> factory;



