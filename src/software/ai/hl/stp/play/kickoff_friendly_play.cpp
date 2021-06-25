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
    
    return (!world.gameState().isStopped() && !world.gameState().isHalted())
        && !receiver_done && world.gameState().isPlaying();
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                         const World &world)
{
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
    //      Part 2: Chip the ball over the defender (ref normal start)

    // the following positions are in the same order as the positions shown above,
    // excluding the goalie for part 1 of this play
    std::vector<Point> kickoff_setup_positions = {
        // Robot 1
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius(), 0)),
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

    // specific tactics
    auto kickoff_move_tactic = std::make_shared<KickoffMoveTactic>(true);
    auto kickoff_chip_tactic = std::make_shared<KickoffChipTactic>(true);

    // Part 1: setup state (move to key positions)
    while (world.gameState().isSetupState())
    {
        PriorityTacticVector result = {{}};

        // set the requirement that Robot 1 must be able to kick and chip
        kickoff_move_tactic->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};
        kickoff_move_tactic->updateControlParams(kickoff_setup_positions.at(0),
                                                Angle::zero(), 0);
        result[0].emplace_back(kickoff_move_tactic);
        
        // setup 5 kickoff positions in order of priority
        for (unsigned i = 1; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                    Angle::zero(), 0);
            result[0].emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }
    
    // These two tactics will set robots to roam around the wings, trying to put
    // themselves into a good position to receive a pass
    std::array<std::shared_ptr<MoveTactic>, 2> cherry_picker_tactics = {
        std::make_shared<MoveTactic>(false),
        std::make_shared<MoveTactic>(false),
    };
    
    std::vector<EighteenZoneId> kickoff_pass_zones = {  EighteenZoneId::ZONE_7, 
                                                        EighteenZoneId::ZONE_9 };
    
    auto update_cherry_pickers = [&](Pass pass1, Pass pass2)
    {        
        std::get<0>(cherry_picker_tactics)->updateControlParams(pass1.receiverPoint(),
                                                                pass1.receiverOrientation(), 0.0,
                                                                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        
        std::get<1>(cherry_picker_tactics)->updateControlParams(pass2.receiverPoint(),
                                                                pass2.receiverOrientation(), 0.0,
                                                                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    };
    
    // Wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score                  = 1.0;
    Timestamp commit_stage_start_time = world.getMostRecentTimestamp();
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());
    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());
    auto pass_eval = pass_generator.generatePassEvaluation(world);
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();
    best_pass = best_pass_and_score_so_far.pass;
    
    do
    {
        PassWithRating pass1 = 
            pass_generator.generatePassEvaluation(world).getBestPassInZones({kickoff_pass_zones[0]});
        PassWithRating pass2 = 
            pass_generator.generatePassEvaluation(world).getBestPassInZones({kickoff_pass_zones[1]});
        update_cherry_pickers(pass1.pass, pass2.pass);
        best_pass_and_score_so_far = (pass1.rating >= pass2.rating) ? pass1 : pass2;

        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                     play_config->getCornerKickPlayConfig()
                                         ->getMaxTimeCommitToPassSeconds()
                                         ->value(),
                                 1.0);
        
        best_pass = best_pass_and_score_so_far.pass;
        Vector ball_to_passer = best_pass.value().receiverPoint() - world.ball().position();
        kickoff_move_tactic->updateControlParams(
            world.ball().position() - ball_to_passer.normalize(ROBOT_MAX_RADIUS_METERS * 1.5),
            ball_to_passer.orientation(), 0);
        
        yield({{kickoff_move_tactic, 
                std::get<0>(cherry_picker_tactics), 
                std::get<1>(cherry_picker_tactics)}});
    } while (best_pass_and_score_so_far.rating < min_score);

    receiver   = std::make_shared<ReceiverTactic>(best_pass.value());
    
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };
    
    committed_to_pass = true;
    do
    {
        std::cout << "Yielding receiver\n";
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
    
    
    // Part 2: not normal play, currently ready state (chip the ball)
//    while (!world.gameState().isPlaying())
//    {  
        

        
//        // TODO This needs to be adjusted post field testing, ball needs to land exactly
//        // in the middle of the enemy field
//        kickoff_chip_tactic->updateControlParams(
//            world.ball().position(), Vector(world.field().xLength(), 0).orientation(),
//            BALL_MAX_SPEED_METERS_PER_SECOND - 1);
//        result[0].emplace_back(kickoff_chip_tactic);
//
//        // the robot at position 0 will be closest to the ball, so positions starting from
//        // 1 will be assigned to the rest of the robots
//        for (unsigned i = 1; i < kickoff_setup_positions.size(); i++)
//        {
//            move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
//                                                    Angle::zero(), 0);
//            result[0].emplace_back(move_tactics.at(i));
//        }

        // yield the Tactics this Play wants to run, in order of priority
//        yield(result);
//    }
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
