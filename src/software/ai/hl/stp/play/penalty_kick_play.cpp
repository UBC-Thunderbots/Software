#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

const std::string PenaltyKickPlay::name = "Penalty Kick Play";

std::string PenaltyKickPlay::getName() const
{
    return PenaltyKickPlay::name;
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

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto penalty_shot_tactic = std::make_shared<PenaltyKickTactic>(
        world.ball(), world.field(), world.enemyTeam().goalie(), true);

    auto shooter_setup_move = std::make_shared<PenaltySetupTactic>(true);

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    move_tactic_2->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    // TODO: Remove the FRIENDLY_HALF whitelist once ticket #980 is complete
    move_tactic_2->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    move_tactic_3->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    // TODO: Remove the FRIENDLY_HALF whitelist once ticket #980 is complete
    move_tactic_3->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    move_tactic_4->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    // TODO: Remove the FRIENDLY_HALF whitelist once ticket #980 is complete
    move_tactic_4->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    move_tactic_5->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    // TODO: Remove the FRIENDLY_HALF whitelist once ticket #980 is complete
    move_tactic_5->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);
    move_tactic_6->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    // TODO: Remove the FRIENDLY_HALF whitelist once ticket #980 is complete
    move_tactic_6->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);

    do
    {
        std::vector<std::shared_ptr<Tactic>> tactics_to_run;

        Vector behind_ball_direction =
            (world.ball().position() - world.field().enemyGoalpostPos()).norm();
        Angle shoot_angle =
            (world.field().enemyGoalpostPos() - world.ball().position()).orientation();

        Point behind_ball = world.ball().position() +
                            behind_ball_direction.norm(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                       BALL_MAX_RADIUS_METERS + 0.1);

        // Move all non-shooter robots to the center of the field
        move_tactic_2->updateControlParams(Point(0, 0),
                                           world.field().enemyGoal().orientation(), 0);
        move_tactic_3->updateControlParams(Point(0, 4 * ROBOT_MAX_RADIUS_METERS),
                                           world.field().enemyGoal().orientation(), 0);
        move_tactic_4->updateControlParams(Point(0, -4 * ROBOT_MAX_RADIUS_METERS),
                                           world.field().enemyGoal().orientation(), 0);
        move_tactic_5->updateControlParams(Point(0, 8 * ROBOT_MAX_RADIUS_METERS),
                                           world.field().enemyGoal().orientation(), 0);
        move_tactic_6->updateControlParams(Point(0, -8 * ROBOT_MAX_RADIUS_METERS),
                                           world.field().enemyGoal().orientation(), 0);

        penalty_shot_tactic->updateWorldParams(world.ball(), world.enemyTeam().goalie(),
                                               world.field());
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

// Register this play in the PlayFactory
static TPlayFactory<PenaltyKickPlay> factory;
