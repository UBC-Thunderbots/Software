#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/passer_tactic.h"
#include "software/ai/hl/stp/tactic/patrol_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/pass_generator.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "src/g3log/loglevels.hpp"

using namespace Passing;

const std::string ShootOrPassPlay::name = "Shoot Or Pass Play";

ShootOrPassPlay::ShootOrPassPlay() {}

std::string ShootOrPassPlay::getName() const
{
    return ShootOrPassPlay::name;
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    bool use_shoot_or_pass_instead_of_shoot_or_chip =
        Util::DynamicParameters->getHighLevelStrategyConfig()
            ->UseShootOrPassInsteadOfShootOrChip()
            ->value();

    return use_shoot_or_pass_instead_of_shoot_or_chip && world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world, world.friendlyTeam());
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (!Evaluation::teamHasPossession(world, world.enemyTeam()) ||
            Evaluation::teamPassInProgress(world, world.friendlyTeam()));
}

void ShootOrPassPlay::getNextTactics(TacticCoroutine::push_type &yield)
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

    // Setup the goalie and crease defenders
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    // If the passing is coming from the friendly end, we split the cherry-pickers
    // across the x-axis in the enemy half
    Rectangle cherry_pick_1_target_region(world.field().centerPoint(),
                                          world.field().enemyCornerPos());
    Rectangle cherry_pick_2_target_region(world.field().centerPoint(),
                                          world.field().enemyCornerNeg());

    // Otherwise, the pass is coming from the enemy end, put the two cherry-pickers
    // on the opposite side of the x-axis to wherever the pass is coming from
    if (world.ball().position().x() > -1)
    {
        double y_offset =
            -std::copysign(world.field().yLength() / 2, world.ball().position().y());
        cherry_pick_1_target_region =
            Rectangle(Point(0, world.field().xLength() / 4),
                      Point(world.field().xLength() / 2, y_offset));
        cherry_pick_2_target_region =
            Rectangle(Point(0, world.field().xLength() / 4), Point(0, y_offset));
    }

    std::array<std::shared_ptr<CherryPickTactic>, 2> cherry_pick_tactics = {
        std::make_shared<CherryPickTactic>(world, cherry_pick_1_target_region),
        std::make_shared<CherryPickTactic>(world, cherry_pick_2_target_region)};


    // Have a robot keep trying to take a shot
    Angle min_open_angle_for_shot =
        Angle::ofDegrees(Util::DynamicParameters->getShootOrPassPlayConfig()
                             ->MinOpenAngleForShotDeg()
                             ->value());

    auto shoot_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        min_open_angle_for_shot, std::nullopt, false);

    // Start a PassGenerator that will continuously optimize passes into the enemy half
    // of the field
    PassGenerator pass_generator(world, world.ball().position(),
                                 PassType::RECEIVE_AND_DRIBBLE);
    pass_generator.setTargetRegion(
        Rectangle(Point(0, world.field().yLength() / 2), world.field().enemyCornerNeg()));
    PassWithRating best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

    // Wait for a good pass by starting out only looking for "perfect" passes (with a
    // score of 1) and decreasing this threshold over time
    double min_pass_score_threshold = 1.0;
    // TODO: change this to use the world timestamp (Issue #423)
    Timestamp pass_optimization_start_time = world.ball().lastUpdateTimestamp();
    // This boolean indicates if we're ready to perform a pass
    bool ready_to_pass = false;
    // Whether or not we've set the passer robot in the PassGenerator
    bool set_passer_robot_in_passgenerator = false;

    double abs_min_pass_score =
        Util::DynamicParameters->getShootOrPassPlayConfig()->AbsMinPassScore()->value();
    double pass_score_ramp_down_duration =
        Util::DynamicParameters->getShootOrPassPlayConfig()
            ->PassScoreRampDownDuration()
            ->value();
    do
    {
        updateCreaseDefenderTactics(crease_defender_tactics);
        updateGoalie(goalie_tactic);
        updateShootGoalTactic(shoot_tactic);
        updateCherryPickTactics(cherry_pick_tactics);
        updatePassGenerator(pass_generator);

        LOG(DEBUG) << "Best pass so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "      with score of: " << best_pass_and_score_so_far.rating;

        yield({goalie_tactic, shoot_tactic, std::get<0>(cherry_pick_tactics),
               std::get<0>(crease_defender_tactics), std::get<1>(cherry_pick_tactics),
               std::get<1>(crease_defender_tactics)});

        best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

        // We're ready to pass if we have a robot assigned in the PassGenerator as the
        // passer and the PassGenerator has found a pass above our current threshold
        ready_to_pass = set_passer_robot_in_passgenerator &&
                        best_pass_and_score_so_far.rating > min_pass_score_threshold;

        // If there is a robot assigned to shoot, we assume this is the robot
        // that will be taking the shot
        if (shoot_tactic->getAssignedRobot())
        {
            pass_generator.setPasserRobotId(shoot_tactic->getAssignedRobot()->id());
            set_passer_robot_in_passgenerator = true;
        }

        // If we've assigned a robot as the passer in the PassGenerator, we lower
        // our threshold based on how long the PassGenerator as been running since
        // we set it
        if (set_passer_robot_in_passgenerator)
        {
            Duration time_since_commit_stage_start =
                world.getMostRecentTimestamp() - pass_optimization_start_time;
            min_pass_score_threshold =
                1 - std::min(time_since_commit_stage_start.getSeconds() /
                                 pass_score_ramp_down_duration,
                             1.0 - abs_min_pass_score);
        }
    } while (!ready_to_pass || shoot_tactic->hasShotAvailable());

    // TODO (Issue #636): We should stop the PassGenerator and Cherry-pick tactic here
    //                    to save CPU cycles

    // If the shoot tactic has not finished, then we need to pass, otherwise we are
    // done this play
    if (!shoot_tactic->done())
    {
        // Commit to a pass
        Pass pass = best_pass_and_score_so_far.pass;

        LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "Score of pass we committed to: "
                   << best_pass_and_score_so_far.rating;

        // Perform the pass and wait until the receiver is finished
        auto passer   = std::make_shared<PasserTactic>(pass, world.ball(), false);
        auto receiver = std::make_shared<ReceiverTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(),
            false);
        do
        {
            updateCreaseDefenderTactics(crease_defender_tactics);
            updateGoalie(goalie_tactic);
            passer->updateWorldParams(world.ball());
            passer->updateControlParams(pass);
            receiver->updateWorldParams(world.friendlyTeam(), world.enemyTeam(),
                                        world.ball());
            receiver->updateControlParams(pass);
            yield({goalie_tactic, passer, receiver, std::get<0>(crease_defender_tactics),
                   std::get<1>(crease_defender_tactics)});
        } while (!receiver->done());
    }
    else
    {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

void ShootOrPassPlay::updateCherryPickTactics(
    std::array<std::shared_ptr<CherryPickTactic>, 2> tactics)
{
    for (auto &tactic : tactics)
    {
        tactic->updateWorldParams(world);
    }
}

void ShootOrPassPlay::updateShootGoalTactic(std::shared_ptr<ShootGoalTactic> shoot_tactic)
{
    shoot_tactic->updateWorldParams(world.field(), world.friendlyTeam(),
                                    world.enemyTeam(), world.ball());
}

void ShootOrPassPlay::updatePassGenerator(PassGenerator &pass_generator)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}

void ShootOrPassPlay::updateCreaseDefenderTactics(
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defenders)
{
    for (auto &crease_defender : crease_defenders)
    {
        crease_defender->updateWorldParams(world.ball(), world.field(),
                                           world.friendlyTeam(), world.enemyTeam());
    }
}

void ShootOrPassPlay::updateGoalie(std::shared_ptr<GoalieTactic> goalie)
{
    goalie->updateWorldParams(world.ball(), world.field(), world.friendlyTeam(),
                              world.enemyTeam());
}

// Register this play in the PlayFactory
static TPlayFactory<ShootOrPassPlay> factory;
