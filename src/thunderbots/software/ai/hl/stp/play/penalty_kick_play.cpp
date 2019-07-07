#include "ai/hl/stp/play/penalty_kick_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/penalty_kick_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "shared/constants.h"

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
    return world.gameState().isOurPenalty() && !world.gameState().isStopped() && !world.gameState().isHalted();
}

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto penalty_shot_tactic = std::make_shared<PenaltyKickTactic>(
        world.ball(), world.field(), world.enemyTeam().goalie(), true);
    penalty_shot_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
    penalty_shot_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
    penalty_shot_tactic->addWhitelistedAvoidArea(AvoidArea::ENEMY_DEFENSE_AREA);
    penalty_shot_tactic->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);

    auto shooter_setup_move = std::make_shared<MoveTactic>(true);

    shooter_setup_move->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    shooter_setup_move->addWhitelistedAvoidArea(AvoidArea::ENEMY_DEFENSE_AREA);
    shooter_setup_move->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    shooter_setup_move->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    move_tactic_2->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    move_tactic_2->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    move_tactic_3->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    move_tactic_3->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    move_tactic_4->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    move_tactic_4->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    move_tactic_5->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    move_tactic_5->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);
    move_tactic_6->addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    move_tactic_6->addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);

    Timestamp start_of_shoot = world.getMostRecentTimestamp();

    do
    {
        std::vector<std::shared_ptr<Tactic>> tactics_to_run;


        Vector behind_ball_direction = (world.ball().position() - world.field().enemyGoal()).norm();
        Angle shoot_angle = (world.field().enemyGoal() -  world.ball().position()).orientation();

        Point behind_ball = world.ball().position() + behind_ball_direction.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS + 0.1);

        move_tactic_2->updateParams(Point(0, 0), world.field().enemyGoal().orientation(),
                                    0);
        move_tactic_3->updateParams(Point(0, 4 * ROBOT_MAX_RADIUS_METERS),
                                    world.field().enemyGoal().orientation(), 0);
        move_tactic_4->updateParams(Point(0, -4 * ROBOT_MAX_RADIUS_METERS),
                                    world.field().enemyGoal().orientation(), 0);
        move_tactic_5->updateParams(Point(0, 8 * ROBOT_MAX_RADIUS_METERS),
                                    world.field().enemyGoal().orientation(), 0);
        move_tactic_6->updateParams(Point(0, -8 * ROBOT_MAX_RADIUS_METERS),
                                    world.field().enemyGoal().orientation(), 0);
        penalty_shot_tactic->updateParams(world.ball(), world.enemyTeam().goalie(), world.field(), start_of_shoot);
        shooter_setup_move->updateParams(behind_ball, shoot_angle, 0.0);

        // If we are setting up for penalty kick, move our robots to position
        if(world.gameState().isSetupState()) {
            tactics_to_run.emplace_back(shooter_setup_move);
        }
        else if(world.gameState().isOurPenalty() ) {
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
