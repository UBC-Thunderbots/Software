#include "ai/hl/stp/play/shoot_or_pass_play.h"

#include <g3log/g3log.hpp>

#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/patrol_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/passing/pass_generator.h"
#include "shared/constants.h"
#include "util/logger/custom_logging_levels.h"

using namespace Passing;

const std::string ShootOrPassPlay::name = "Shoot Or Pass Play";

ShootOrPassPlay::ShootOrPassPlay()
    : MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(2.0)),
    MIN_NET_OPEN_ANGLE_FOR_SHOT(Angle::ofDegrees(3))
{
}

std::string ShootOrPassPlay::getName() const
{
    return ShootOrPassPlay::name;
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return Evaluation::teamHasPossession(world.friendlyTeam(), world.ball());
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return Evaluation::teamHasPossession(world.friendlyTeam(), world.ball());
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

    // Have two robots cherry-pick on the +y and -y sides of the field
    auto cherry_pick_tactic_pos_y = std::make_shared<CherryPickTactic>(
        world, Rectangle(world.field().centerPoint(), world.field().enemyCornerPos()));
    auto cherry_pick_tactic_neg_y = std::make_shared<CherryPickTactic>(
        world, Rectangle(world.field().centerPoint(), world.field().enemyCornerNeg()));

    // Have two robots patrol along a line near the sides of the field
    // TODO: make these two patrollers into "crease defenders"
    double half_field_width        = world.field().width() / 2;
    double patrol_point_y_position = half_field_width - half_field_width / 4;
    auto patrol_tactic_pos_y =
        std::make_shared<PatrolTactic>(std::vector<Point>({
                                           Point(0, patrol_point_y_position),
                                           Point(1, patrol_point_y_position),
                                       }),
                                       AT_PATROL_POINT_TOLERANCE, SPEED_AT_PATROL_POINTS);
    auto patrol_tactic_neg_y =
        std::make_shared<PatrolTactic>(std::vector<Point>({
                                           Point(0, -patrol_point_y_position),
                                           Point(1, -patrol_point_y_position),
                                       }),
                                       AT_PATROL_POINT_TOLERANCE, SPEED_AT_PATROL_POINTS);

    // Have a robot keep trying to take a shot
    auto shoot_tactic = std::make_shared<ShootGoalTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), MIN_NET_OPEN_ANGLE_FOR_SHOT, std::nullopt, false);

    // Start a PassGenerator that will continuously optimize passes into roughly
    // the enemy half of the field
    PassGenerator pass_generator(world, world.ball().position(), PassType::RECEIVE_AND_DRIBBLE);
    pass_generator.setTargetRegion(Rectangle(
            Point(-(world.field().length() / 4), world.field().width()/2),
            world.field().enemyCornerNeg()
            ));
    std::pair<Pass, double> best_pass_and_score_so_far =
        pass_generator.getBestPassSoFar();

    // Wait for a good pass by starting out only looking for "perfect" passes (with a
    // score of 1) and decreasing this threshold over time
    double min_pass_score_threshold = 1.0;
    // TODO: change this to use the world timestamp (Issue #423)
    Timestamp pass_optimization_start_time = world.ball().lastUpdateTimestamp();
    // This boolean indicates if we're ready to perform a pass
    bool ready_to_pass = false;
    // Whether or not we've set the passer robot in the PassGenerator
    bool set_passer_robot_in_passgenerator = false;

    do {

        // TODO: Update "crease defenders" here when they're done
        updateShootGoalTactic(shoot_tactic);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);

        LOG(DEBUG) << "Best pass so far is: " << best_pass_and_score_so_far.first;
        LOG(DEBUG) << "      with score of: " << best_pass_and_score_so_far.second;

        yield({shoot_tactic, cherry_pick_tactic_neg_y, /* cherry_pick_tactic_pos_y, patrol_tactic_pos_y, patrol_tactic_neg_y*/});

        // If there is a robot assigned to shoot, we assume this is the robot
        // that will be taking the shot
        if (shoot_tactic->getAssignedRobot()){
            pass_generator.setPasserRobotId(shoot_tactic->getAssignedRobot()->id());
            set_passer_robot_in_passgenerator = true;
        }

        best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

        // We're ready to pass if we have a robot assigned in the PassGenerator as the
        // passer and the PassGenerator has found a pass above our current threshold
        ready_to_pass = set_passer_robot_in_passgenerator && best_pass_and_score_so_far.second < min_pass_score_threshold;

        // If we've assigned a robot as the passer in the PassGenerator, we lower
        // our threshold based on how long the PassGenerator as been running since
        // we set it
        if (set_passer_robot_in_passgenerator){

            // TODO: change this to use the world timestamp (Issue #423)
            Duration time_since_commit_stage_start =
                    world.ball().lastUpdateTimestamp() - pass_optimization_start_time;
            min_pass_score_threshold =
                    1 - std::min(time_since_commit_stage_start.getSeconds() /
                                 MAX_TIME_TO_COMMIT_TO_PASS.getSeconds(),
                                 1.0 - ABS_MIN_PASS_QUALITY);
        }

        LOG(DEBUG) << "LOOP END";
//    } while(!ready_to_pass || shoot_tactic->hasShotAvailable());
    } while(!ready_to_pass || false);

    // Destruct the PassGenerator and CherryPick tactics (which contain a PassGenerator
    // each) to save a significant number of CPU cycles
    // TODO: stop the PassGenerators here instead of destructing them (Issue #636)
    pass_generator.~PassGenerator();
    cherry_pick_tactic_pos_y->~CherryPickTactic();
    cherry_pick_tactic_neg_y->~CherryPickTactic();

    // If the shoot tactic has finished, we are done this play, otherwise we need to pass
//    if (!shoot_tactic->hasShotAvailable()){
    if (true){
        // Commit to a pass
        Pass pass = best_pass_and_score_so_far.first;

        LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.first;
        LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.second;

        // Perform the pass and wait until the receiver is finished
        auto passer = std::make_shared<PasserTactic>(pass, world.ball(), false);
        auto receiver =
                std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(),
                                                 world.enemyTeam(), pass, world.ball(), false);
        do
        {
            // TODO: Update "crease defenders" here when they're done
            passer->updateParams(pass, world.ball());
            receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass,
                                   world.ball());
            yield({passer, receiver, patrol_tactic_pos_y, patrol_tactic_neg_y});
        } while (!receiver->done());
    } else {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

void ShootOrPassPlay::updateCherryPickTactics(
    std::vector<std::shared_ptr<CherryPickTactic>> tactics)
{
    for (auto &tactic : tactics)
    {
        tactic->updateParams(world);
    }
}

void ShootOrPassPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic)
{
    // We want to the robot to face the enemy net to minimize the amount of motion
    // required to turn and shoot
    Vector ball_to_enemy_net_vec = world.field().enemyGoal() - world.ball().position();
    align_to_ball_tactic->updateParams(
        world.ball().position() - ball_to_enemy_net_vec.norm(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_enemy_net_vec.orientation(), 0);
}

void ShootOrPassPlay::updateShootGoalTactic(std::shared_ptr<ShootGoalTactic> shoot_tactic){
    shoot_tactic->updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball());
}

void ShootOrPassPlay::updatePassGenerator(PassGenerator &pass_generator)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}

// Register this play in the PlayFactory
static TPlayFactory<ShootOrPassPlay> factory;
