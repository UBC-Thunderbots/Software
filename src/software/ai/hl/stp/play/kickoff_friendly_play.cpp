#include "software/ai/hl/stp/play/kickoff_friendly_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"
#include "software/ai/hl/stp/tactic/kickoff_move_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

KickoffFriendlyPlay::KickoffFriendlyPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
{
}

bool KickoffFriendlyPlay::isApplicable(const World &world) const
{    
    return world.gameState().isOurKickoff()
        && (world.gameState().isSetupState())
        && !world.gameState().isHalted() && !world.gameState().isStopped();
}

bool KickoffFriendlyPlay::invariantHolds(const World &world) const
{   
    bool receiver_done = false;
    
    if (receiver)
    {
        receiver_done = receiver->done();
    }
    
    return (!world.gameState().isHalted() &&
            !world.gameState().isStopped() && world.gameState().isOurKickoff() && !world.gameState().isPlaying()) && !receiver_done;
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                         const World &world)
{
    using Zones = EighteenZoneId;
    
    // Since we only have 6 robots at the maximum, the number one priority
    // is the robot doing the kickoff up front. The goalie is the second most
    // important, followed by 3 and 4 setup for offense. 5 and 6 will stay
    // back near the goalie just in case the ball quickly returns to the friendly
    // side of the field.
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|               3    |                    |
    // 		|                    |                    |
    // 		+--+ 5               |                 +--+
    // 		|  |                 |                 |  |
    // 		|  |               +-+-+               |  |
    // 		|2 |               |1  |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+ 6               |                 +--+
    // 		|                    |                    |
    // 		|               4    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+
    //
    // This is a two part play:
    //      Part 1: Get into position, but don't touch the ball (ref kickoff)
    //      Part 2: Pass the ball to a winger robot (ref normal start)

    // the following positions are in the same order as the positions shown above,
    // excluding the goalie for part 1 of this play
    std::vector<Point> kickoff_setup_positions = {
        // Robot 1
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius() + 0.1, 0)),
        // Robot 2
        // Goalie positions will be handled by the goalie tactic
        // Robot 3
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                     -1.0 / 3.0 * world.field().yLength())),
        // Robot 4
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                     1.0 / 3.0 * world.field().yLength())),
        // Robot 5
        Point(world.field().friendlyGoalpostPos().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostPos().y()),
        // Robot 6
        Point(world.field().friendlyGoalpostNeg().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostNeg().y()),
    };

    // move tactics to use to move to positions defined above
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    // specific tactics for kickoff robots (special motion constraints for moving in centre circle)
    auto kickoff_move_tactic = std::make_shared<KickoffMoveTactic>(true);
    auto kickoff_chip_tactic = std::make_shared<KickoffChipTactic>(true);

        
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };
    
    // Part 1: setup state (move to key positions)
    while (world.gameState().isSetupState())
    {
        PriorityTacticVector result = {{}};

        // set the requirement that Robot 1 must be able to kick and chip
        kickoff_move_tactic->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};
        
        // setup 5 kickoff positions in order of priority
        for (unsigned i = 0; i < kickoff_setup_positions.size() - 2; i++)
        {
            move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                    Angle::zero(), 0);
            result[0].emplace_back(move_tactics.at(i));
        }
        
        result[0].emplace_back(crease_defender_tactics[0]);
        result[0].emplace_back(crease_defender_tactics[1]);
        
        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }
    
    // These two tactics will set robots to roam around the wings, trying to put
    // themselves into a good position to receive a pass
    std::array<std::shared_ptr<MoveTactic>, 2> winger_tactics = {
        std::make_shared<MoveTactic>(false),
        std::make_shared<MoveTactic>(false),
    };
    
    std::vector<EighteenZoneId> kickoff_pass_zones = {  EighteenZoneId::ZONE_7, 
                                                        EighteenZoneId::ZONE_10,
                                                        EighteenZoneId::ZONE_9,
                                                        EighteenZoneId::ZONE_12
                                                        };
    
    auto update_wingers = [&](Pass left, Pass right)
    {        
        std::get<0>(winger_tactics)->updateControlParams(left.receiverPoint(),
                                                                left.receiverOrientation(), 0.0,
                                                                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        
        std::get<1>(winger_tactics)->updateControlParams(right.receiverPoint(),
                                                                right.receiverOrientation(), 0.0,
                                                                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    };
    
    // Part 2: Wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score                  = 1.0;
    Timestamp commit_stage_start_time = world.getMostRecentTimestamp();
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());
    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());
    auto pass_eval = pass_generator.generatePassEvaluation(world);
    PassWithRating best_pass_and_score_so_far = 
        pass_eval.getBestPassInZones(std::unordered_set<Zones>(kickoff_pass_zones.begin(), 
                                                               kickoff_pass_zones.end()));

    do
    {
        PassWithRating pass1 = 
            pass_generator.generatePassEvaluation(world).getBestPassInZones({kickoff_pass_zones[0], kickoff_pass_zones[1]});
        PassWithRating pass2 = 
            pass_generator.generatePassEvaluation(world).getBestPassInZones({kickoff_pass_zones[2], kickoff_pass_zones[3]});
        update_wingers(pass1.pass, pass2.pass);
        
        auto current_best_pass_with_rating = (pass1.rating >= pass2.rating) ? pass1 : pass2;

        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                     play_config->getCornerKickPlayConfig()
                                         ->getMaxTimeCommitToPassSeconds()
                                         ->value(),
                                 1.0);
        
        
        // to prevent the kicker from being paralyzed at kickoff, we accept a pass threshold
        // where we lock on to a less favourable pass
        if ((best_pass_and_score_so_far.rating + BETTER_PASS_RATING_THRESHOLD) 
            < current_best_pass_with_rating.rating)
        {
            best_pass_and_score_so_far = current_best_pass_with_rating;
        }
        best_pass = best_pass_and_score_so_far.pass;
        
        Vector ball_to_passer = best_pass.value().receiverPoint() - world.ball().position();
        kickoff_move_tactic->updateControlParams(
            world.ball().position() - ball_to_passer.normalize(ROBOT_MAX_RADIUS_METERS),
            ball_to_passer.orientation(), 0);
        yield({{kickoff_move_tactic, 
                std::get<0>(winger_tactics), 
                std::get<1>(winger_tactics)}});
    } while (best_pass_and_score_so_far.rating < min_score);

    // part 2: we are ready to pass, let's make one now
    receiver   = std::make_shared<ReceiverTactic>(best_pass.value());
    
    committed_to_pass = true;
    do
    {
        kickoff_chip_tactic->updateControlParams(best_pass.value().passerPoint(),
                                                best_pass.value().passerOrientation(),
                                                best_pass.value().speed());
        receiver->updateControlParams(best_pass.value());
        
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::RIGHT);
        yield({{kickoff_chip_tactic, receiver, std::get<0>(crease_defender_tactics),
               std::get<1>(crease_defender_tactics)}});
    } while (!receiver->done());
}


std::vector<CircleWithColor> KickoffFriendlyPlay::getCirclesWithColorToDraw()
{
    if (best_pass.has_value())
    {
        if (committed_to_pass)
        {
            return {std::make_pair<Circle, std::string>(
                Circle(best_pass->receiverPoint(), 0.2), "blue")};
        }
        return {std::make_pair<Circle, std::string>(
            Circle(best_pass->receiverPoint(), 0.20), "pink")};
    }
    return {};
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffFriendlyPlay, PlayConfig> factory;
