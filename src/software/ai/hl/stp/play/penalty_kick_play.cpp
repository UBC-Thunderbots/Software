#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

PenaltyKickPlay::PenaltyKickPlay(std::shared_ptr<const PlayConfig> config)
{
    play_config = config;
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

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    do
    {
        std::vector<std::shared_ptr<Tactic>> tactics_to_run;

        Vector behind_ball_direction =
            (world.ball().position() - world.field().enemyGoalpostPos()).normalize();
        Angle shoot_angle =
            (world.field().enemyGoalpostPos() - world.ball().position()).orientation();

        Point behind_ball = world.ball().position() + behind_ball_direction.normalize(
                                                          DIST_TO_FRONT_OF_ROBOT_METERS +
                                                          BALL_MAX_RADIUS_METERS + 0.1);

        // Move all non-shooter robots to the center of the field
        move_tactic_2->updateControlParams(
            Point(0, 0), world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_3->updateControlParams(
            Point(0, 4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_4->updateControlParams(
            Point(0, -4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_5->updateControlParams(
            Point(0, 8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_6->updateControlParams(
            Point(0, -8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);

        shooter_setup_move->updateControlParams(behind_ball, shoot_angle, 0.0);

        // If we are setting up for penalty kick, move our robots to position
        if (world.gameState().isSetupState())
        {
            tactics_to_run.emplace_back(shooter_setup_move);
        }
        else if (world.gameState().isOurPenalty())
        {
            tactics_to_run.emplace_back(penalty_shot_tactic);
        }
        // Move all non-shooter robots to the center of the field

        tactics_to_run.emplace_back(move_tactic_2);
        tactics_to_run.emplace_back(move_tactic_3);
        tactics_to_run.emplace_back(move_tactic_4);
        tactics_to_run.emplace_back(move_tactic_5);
        tactics_to_run.emplace_back(move_tactic_6);

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickPlay, PlayConfig> factory;
