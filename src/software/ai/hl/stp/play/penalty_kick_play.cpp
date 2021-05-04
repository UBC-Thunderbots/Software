#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

PenaltyKickPlay::PenaltyKickPlay(std::shared_ptr<const PlayConfig> config) : Play(config)
{
}

bool PenaltyKickPlay::isApplicable(const World &world) const
{
    return (world.gameState().isReadyState() || world.gameState().isSetupState()) &&
           world.gameState().isOurPenalty();
}

bool PenaltyKickPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurPenalty() && !world.gameState().isStopped() &&
           !world.gameState().isHalted();
}

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
{
    auto penalty_shot_tactic = std::make_shared<PenaltyKickTactic>(
        world.ball(), world.field(), world.enemyTeam().goalie(), true);

    auto shooter_setup_move = std::make_shared<PenaltySetupTactic>(true);

    // Setup the goalie
    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig());

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);

    do
    {
        PriorityTacticVector tactics_to_run = {{}};

        Vector behind_ball_direction =
            (world.ball().position() - world.field().enemyGoalpostPos()).normalize();
        Angle shoot_angle =
            (world.field().enemyGoalpostPos() - world.ball().position()).orientation();

        Point behind_ball = world.ball().position() + behind_ball_direction.normalize(
                                                          DIST_TO_FRONT_OF_ROBOT_METERS +
                                                          BALL_MAX_RADIUS_METERS + 0.1);
        double ball_position_x = world.field().friendlyPenaltyMark().x();

        // Move all non-shooter robots to a position behind the ball
        move_tactic_2->updateControlParams(
            Point(ball_position_x - 1.25, 0),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_3->updateControlParams(
            Point(ball_position_x - 1.25, 4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_4->updateControlParams(
            Point(ball_position_x - 1.25, -4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_5->updateControlParams(
            Point(ball_position_x - 1.25, 8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);

        shooter_setup_move->updateControlParams(behind_ball, shoot_angle, 0.0);

        // If we are setting up for penalty kick, move our robots to position
        if (world.gameState().isSetupState())
        {
            tactics_to_run[0].emplace_back(shooter_setup_move);
        }
        else if (world.gameState().isOurPenalty())
        {
            tactics_to_run[0].emplace_back(penalty_shot_tactic);
        }

        tactics_to_run[0].emplace_back(goalie_tactic);
        // Move all non-shooter robots behind the ball
        tactics_to_run[0].emplace_back(move_tactic_2);
        tactics_to_run[0].emplace_back(move_tactic_3);
        tactics_to_run[0].emplace_back(move_tactic_4);
        tactics_to_run[0].emplace_back(move_tactic_5);

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickPlay, PlayConfig> factory;
