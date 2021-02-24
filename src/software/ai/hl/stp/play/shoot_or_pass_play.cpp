#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/passer_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/pass_generator.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config)
    : play_config(config)
{
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

void ShootOrPassPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
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
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(), play_config->getGoalieTacticConfig());
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    // Have a robot keep trying to take a shot
    Angle min_open_angle_for_shot = Angle::fromDegrees(play_config->getShootOrPassPlayConfig()
                                                           ->getMinOpenAngleForShotDeg()
                                                           ->value());

    auto shoot_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        min_open_angle_for_shot, std::nullopt, false, play_config->getShootGoalTacticConfig());

    PassWithRating best_pass_and_score_so_far = attemptToShootWhileLookingForAPass(
        yield, goalie_tactic, crease_defender_tactics, shoot_tactic, world);

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
        auto passer =
            std::make_shared<PasserTactic>(pass, world.ball(), world.field(), false);
        auto receiver = std::make_shared<ReceiverTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(),
            false);
        do
        {
            passer->updateControlParams(pass);
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

PassWithRating ShootOrPassPlay::attemptToShootWhileLookingForAPass(
    TacticCoroutine::push_type &yield, std::shared_ptr<GoalieTactic> goalie_tactic,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<ShootGoalTactic> shoot_tactic, const World &world)
{
    // If the passing is coming from the friendly end, we split the cherry-pickers
    // across the x-axis in the enemy half
    Rectangle cherry_pick_1_target_region = world.field().enemyPositiveYQuadrant();
    Rectangle cherry_pick_2_target_region = world.field().enemyNegativeYQuadrant();

    // Otherwise, the pass is coming from the enemy end, put the two cherry-pickers
    // on the opposite side of the x-axis to wherever the pass is coming from
    if (world.ball().position().x() > -1)
    {
        double y_offset =
            -std::copysign(world.field().yLength() / 2, world.ball().position().y());
        cherry_pick_1_target_region =
            Rectangle(Point(0, 0), Point(world.field().xLength() / 4, y_offset));
        cherry_pick_2_target_region =
            Rectangle(Point(world.field().xLength() / 4, 0),
                      Point(world.field().xLength() / 2, y_offset));
    }

    std::array<std::shared_ptr<CherryPickTactic>, 2> cherry_pick_tactics = {
        std::make_shared<CherryPickTactic>(world, cherry_pick_1_target_region,
                                           play_config->getPassingConfig()),
        std::make_shared<CherryPickTactic>(world, cherry_pick_2_target_region,
                                           play_config->getPassingConfig())};

    // Start a PassGenerator that will continuously optimize passes into the enemy half
    // of the field
    PassGenerator pass_generator(world, world.ball().position(),
                                 PassType::RECEIVE_AND_DRIBBLE,
                                 play_config->getPassingConfig());
    pass_generator.setTargetRegion(world.field().enemyHalf());
    PassWithRating best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

    // Wait for a good pass by starting out only looking for "perfect" passes (with a
    // score of 1) and decreasing this threshold over time
    double min_pass_score_threshold        = 1.0;
    Timestamp pass_optimization_start_time = world.getMostRecentTimestamp();
    // This boolean indicates if we're ready to perform a pass
    bool ready_to_pass = false;
    // Whether or not we've set the passer robot in the PassGenerator
    bool set_passer_robot_in_passgenerator = false;

    double abs_min_pass_score = play_config
                                    ->getShootOrPassPlayConfig()
                                    ->getAbsMinPassScore()
                                    ->value();
    double pass_score_ramp_down_duration = play_config
                                               ->getShootOrPassPlayConfig()
                                               ->getPassScoreRampDownDuration()
                                               ->value();
    do
    {
        updatePassGenerator(pass_generator, world);

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
                1 - std::min(time_since_commit_stage_start.toSeconds() /
                                 pass_score_ramp_down_duration,
                             1.0 - abs_min_pass_score);
        }
    } while (!ready_to_pass || shoot_tactic->hasShotAvailable());
    return best_pass_and_score_so_far;
}

void ShootOrPassPlay::updatePassGenerator(PassGenerator &pass_generator,
                                          const World &world)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}


// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, PlayConfig> factory;
