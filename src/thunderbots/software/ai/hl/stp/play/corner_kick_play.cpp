#include "ai/hl/stp/play/corner_kick_play.h"

#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/passing/pass_generator.h"
#include "shared/constants.h"

using namespace AI::Passing;

const std::string CornerKickPlay::name = "CornerKick Play";

std::string CornerKickPlay::getName() const
{
    return CornerKickPlay::name;
}

bool CornerKickPlay::isApplicable(const World &world) const
{
    // TODO
    return true;
}

bool CornerKickPlay::invariantHolds(const World &world) const
{
    return Evaluation::teamHasPossession(world.friendlyTeam(), world.ball());
}

void CornerKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    //auto cherry_pick_tactic_1 = std::make_shared<CherryPickTactic>(world, Rectangle(world.field().friendlyCornerPos(), world.field().enemyGoal()), true);
    //auto cherry_pick_tactic_1 = std::make_shared<CherryPickTactic>(world, Rectangle({-0.5, -0.5}, {0.5, 0.5}), true);
    auto cherry_pick_tactic_1 = std::make_shared<CherryPickTactic>(world, world.field().enemyDefenseArea(), true);

    PassGenerator pass_generator(world, world.ball().position());

    std::pair<Pass, double> best_pass_and_score_so_far = pass_generator.getBestPassSoFar();

    // Wait for a robot to be assigned to align to take the corner
    while(!align_to_ball_tactic->getAssignedRobot()){
        updateAlignToBallTactic(align_to_ball_tactic);
        updateCherryPickTactics({cherry_pick_tactic_1});
        updatePassGenerator(pass_generator);

        yield({align_to_ball_tactic, cherry_pick_tactic_1});
    }

    std::cout << "ASSIGNED" << std::endl;
    // Set the passer on the pass generator
    pass_generator.setPasserRobotId(align_to_ball_tactic->getAssignedRobot()->id());

    // Align the kicker to take the corner kick and wait for a good pass
    do {
        // Continue trying to align to deal with the case where we've finished
        // aligning but there's not passing to
        if (align_to_ball_tactic->done()){
            align_to_ball_tactic = std::make_shared<MoveTactic>();
        }
        updateAlignToBallTactic(align_to_ball_tactic);
        updateCherryPickTactics({cherry_pick_tactic_1});
        updatePassGenerator(pass_generator);

        yield({align_to_ball_tactic, cherry_pick_tactic_1});

        best_pass_and_score_so_far = pass_generator.getBestPassSoFar();
        std::cout << best_pass_and_score_so_far.second << std::endl;

    } while(!align_to_ball_tactic->done() || best_pass_and_score_so_far.second < MIN_PASS_SCORE);

    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.first;

    std::cout << "COMMITING TO: " << pass << std::endl;

    // TODO: need to stop the pass generator here, need start/stop methods

    // TODO: send the remaining robots forward in the hope we'll make opportunities

    // Perform the pass
    auto passer = std::make_shared<PasserTactic>(pass, world.ball(), false);
    auto receiver = std::make_shared<ReceiverTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(), false);
    while(true){
        passer->updateParams(pass, world.ball());
        receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());

        yield({passer, receiver});
    }
    volatile  int a;
}

void CornerKickPlay::updateCherryPickTactics(
        std::vector<std::shared_ptr<CherryPickTactic>> tactics) {
    for (auto& tactic : tactics){
        tactic->updateParams(world);
    }
}

void CornerKickPlay::updateAlignToBallTactic(
        std::shared_ptr<MoveTactic> align_to_ball_tactic) {
    Vector ball_to_center_vec = Vector(0,0) - world.ball().position();
// We want the kicker to get into position behind the ball facing the center
// of the field
    align_to_ball_tactic->updateParams(
            world.ball().position() - ball_to_center_vec.norm(ROBOT_MAX_RADIUS_METERS*2), ball_to_center_vec.orientation(), 0);
}

void CornerKickPlay::updatePassGenerator(PassGenerator &pass_generator) {
    pass_generator.setWorld(world);
//    pass_generator.setPasserPoint(world.ball().position());
}

// Register this play in the PlayFactory
static TPlayFactory<CornerKickPlay> factory;
