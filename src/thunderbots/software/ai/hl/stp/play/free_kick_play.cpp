#include "ai/hl/stp/play/free_kick_play.h"

#include <g3log/g3log.hpp>

#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/patrol_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/passing/pass_generator.h"
#include "shared/constants.h"
#include "util/logger/custom_logging_levels.h"

using namespace Passing;

const std::string FreeKickPlay::name = "Free Kick Play";

namespace
{
    const Angle MIN_OPEN_NET_ANGLE = Angle::ofDegrees(6);
}

FreeKickPlay::FreeKickPlay()
    : MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(2.0)),
      MIN_NET_OPEN_ANGLE_FOR_SHOT(Angle::ofDegrees(3))
{
}

std::string FreeKickPlay::getName() const
{
    return FreeKickPlay::name;
}

bool FreeKickPlay::isApplicable(const World &world) const
{
    // use this play if it's our indirect or if it's a goal kick (our direct on friendly
    // side)
    //    return world.gameState().isOurIndirectFree() ||
    //           (world.gameState().isOurDirectFree() && world.ball().position().x() < 0);
    return false;
}

bool FreeKickPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurIndirectFree() ||
           (world.gameState().isOurDirectFree() && world.ball().position().x() < 0);
}

void FreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    /**
     * There are two main stages to this Play:
     * 1. Shoot while optimizing passes
     *  - In this stage we try our best to shoot, while also optimizing passes
     *  - Two robots move up to cherry-pick, two stay back as defenders, one is the
     *    shooter/potential passer
     * 2. If we could not shoot, perform the best pass we currently know about
     *  - In this stage the shooter should be re-assigned to be a passer, one of
     *    the cherry-pick tactics should be re-assigned to be a receiver, and the
     *    two defenders continue to defend
     */

    // Have two robots cherry-pick on the +y and -y sides of the field
    auto cherry_pick_tactic_pos_y = std::make_shared<CherryPickTactic>(
        world, Rectangle(world.field().centerPoint(), world.field().enemyCornerPos()));
    auto cherry_pick_tactic_neg_y = std::make_shared<CherryPickTactic>(
        world, Rectangle(world.field().centerPoint(), world.field().enemyCornerNeg()));

    // Setup crease defenders
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT)};

    // Setup goalie
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    // Start a PassGenerator that will continuously optimize passes into roughly
    // the enemy half of the field
    PassGenerator pass_generator(world, world.ball().position());
    pass_generator.setTargetRegion(
        Rectangle(Point(-(world.field().length() / 4), world.field().width() / 2),
                  world.field().enemyCornerNeg()));
    std::pair<Pass, double> best_pass_and_score_so_far =
        pass_generator.getBestPassSoFar();

    // Wait for a good pass by starting out only looking for "perfect" passes (with a
    // score of 1) and decreasing this threshold over time
    double min_pass_score_threshold        = 1.0;
    Timestamp pass_optimization_start_time = world.getMostRecentTimestamp();
    // This boolean indicates if we're ready to perform a pass
    bool ready_to_pass = false;
    // Whether or not we've set the passer robot in the PassGenerator
    bool set_passer_robot_in_passgenerator = false;

    // Wait for a robot to be assigned to align to take the kick
    while (!align_to_ball_tactic->getAssignedRobot())
    {
        LOG(DEBUG) << "Nothing assigned to align to ball yet";
        updateAlignToBallTactic(align_to_ball_tactic);
        align_to_ball_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        align_to_ball_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);
        goalie_tactic->updateParams(world.ball(), world.field(), world.friendlyTeam(),
                                    world.enemyTeam());

        yield({goalie_tactic, align_to_ball_tactic, cherry_pick_tactic_pos_y,
               cherry_pick_tactic_neg_y, crease_defender_tactics[0],
               crease_defender_tactics[1]});
    }

    // Put the robot in roughly the right position to perform the kick
    LOG(DEBUG) << "Aligning to ball";
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic);
        align_to_ball_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        align_to_ball_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);
        goalie_tactic->updateParams(world.ball(), world.field(), world.friendlyTeam(),
                                    world.enemyTeam());


        yield({goalie_tactic, align_to_ball_tactic, cherry_pick_tactic_pos_y,
               cherry_pick_tactic_neg_y, crease_defender_tactics[0],
               crease_defender_tactics[1]});
    } while (!align_to_ball_tactic->done());

    LOG(DEBUG) << "Finished aligning to ball";

    // Have a robot keep trying to take a shot
    auto shoot_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        MIN_NET_OPEN_ANGLE_FOR_SHOT, std::nullopt, false);

    do
    {
        updateDefendersAndGoalie(crease_defender_tactics, goalie_tactic, world);
        updateShootGoalTactic(shoot_tactic);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);
        shoot_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        shoot_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
        goalie_tactic->updateParams(world.ball(), world.field(), world.friendlyTeam(),
                                    world.enemyTeam());

        LOG(DEBUG) << "Best pass so far is: " << best_pass_and_score_so_far.first;
        LOG(DEBUG) << "      with score of: " << best_pass_and_score_so_far.second;

        yield({goalie_tactic, shoot_tactic, cherry_pick_tactic_neg_y,
               cherry_pick_tactic_pos_y, crease_defender_tactics[0],
               crease_defender_tactics[1]});
        // If there is a robot assigned to shoot, we assume this is the robot
        // that will be taking the shot
        if (shoot_tactic->getAssignedRobot())
        {
            pass_generator.setPasserRobotId(shoot_tactic->getAssignedRobot()->id());
            set_passer_robot_in_passgenerator = true;
        }

        best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

        // We're ready to pass if we have a robot assigned in the PassGenerator as the
        // passer and the PassGenerator has found a pass above our current threshold
        ready_to_pass = set_passer_robot_in_passgenerator &&
                        best_pass_and_score_so_far.second < min_pass_score_threshold;

        // If we've assigned a robot as the passer in the PassGenerator, we lower
        // our threshold based on how long the PassGenerator as been running since
        // we set it
        if (set_passer_robot_in_passgenerator)
        {
            Duration time_since_commit_stage_start =
                world.getMostRecentTimestamp() - pass_optimization_start_time;
            min_pass_score_threshold =
                1 - std::min(time_since_commit_stage_start.getSeconds() /
                                 MAX_TIME_TO_COMMIT_TO_PASS.getSeconds(),
                             1.0 - ABS_MIN_PASS_QUALITY);
        }

        LOG(DEBUG) << "LOOP END";
    } while (!ready_to_pass || shoot_tactic->hasShotAvailable());

    // TODO (Issue #636): We should stop the PassGenerator and Cherry-pick tactic here
    //                    to save CPU cycles

    bool kick_from_pos_corner     = world.ball().position().y() > 0;
    Point opposite_corner_to_kick = kick_from_pos_corner ? world.field().enemyCornerNeg()
                                                         : world.field().enemyCornerPos();
    Point bait_move_tactic_1_pos =
        opposite_corner_to_kick - Vector(world.field().enemyDefenseArea().width() * 0.5,
                                         copysign(0.5, opposite_corner_to_kick.y()));


    auto bait_move_tactic_1 = std::make_shared<MoveTactic>(true);
    // If the shoot tactic has finished, we are done this play, otherwise we need to pass
    if (!shoot_tactic->hasShotAvailable())
    {
        // Commit to a pass
        Pass pass = best_pass_and_score_so_far.first;

        LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.first;
        LOG(DEBUG) << "Score of pass we committed to: "
                   << best_pass_and_score_so_far.second;

        // Perform the pass and wait until the receiver is finished
        auto passer   = std::make_shared<PasserTactic>(pass, world.ball(), false);
        auto receiver = std::make_shared<ReceiverTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(),
            false);
        do
        {
            updateDefendersAndGoalie(crease_defender_tactics, goalie_tactic, world);
            passer->updateParams(pass, world.ball());
            receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass,
                                   world.ball());
            bait_move_tactic_1->updateParams(
                bait_move_tactic_1_pos,
                (world.field().enemyGoal() - bait_move_tactic_1_pos).orientation(), 0.0);

            yield({goalie_tactic, passer, receiver, crease_defender_tactics[0],
                   crease_defender_tactics[1], bait_move_tactic_1});
        } while (!receiver->done());
    }
    else
    {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

void FreeKickPlay::updateCherryPickTactics(
    std::vector<std::shared_ptr<CherryPickTactic>> tactics)
{
    for (auto &tactic : tactics)
    {
        tactic->updateParams(world);
    }
}

void FreeKickPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic)
{
    // We want to the robot to face the enemy net to minimize the amount of motion
    // required to turn and shoot
    Vector ball_to_enemy_net_vec = world.field().enemyGoal() - world.ball().position();
    align_to_ball_tactic->updateParams(
        world.ball().position() - ball_to_enemy_net_vec.norm(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_enemy_net_vec.orientation(), 0);
}

void FreeKickPlay::updateShootGoalTactic(std::shared_ptr<ShootGoalTactic> shoot_tactic)
{
    shoot_tactic->updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                               world.ball());
}

void FreeKickPlay::updatePassGenerator(PassGenerator &pass_generator)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}

void FreeKickPlay::updateDefendersAndGoalie(
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<GoalieTactic> goalie_tactic, const World &world)
{
    // If we have any crease defenders, we don't want the goalie tactic to consider
    // them when deciding where to block
    Team friendly_team_for_goalie = world.friendlyTeam();
    for (auto crease_defender_tactic : crease_defender_tactics)
    {
        if (crease_defender_tactic->getAssignedRobot())
        {
            friendly_team_for_goalie.removeRobotWithId(
                crease_defender_tactic->getAssignedRobot()->id());
        }
    }
    goalie_tactic->updateParams(world.ball(), world.field(), friendly_team_for_goalie,
                                world.enemyTeam());
    // Update crease defenders
    for (auto &crease_defender_tactic : crease_defender_tactics)
    {
        crease_defender_tactic->updateParams(world.ball(), world.field(),
                                             world.friendlyTeam(), world.enemyTeam());
    }
}
// Register this play in the PlayFactory
static TPlayFactory<FreeKickPlay> factory;
