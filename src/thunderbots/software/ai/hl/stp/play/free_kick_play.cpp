#include "ai/hl/stp/play/free_kick_play.h"

#include <ai/hl/stp/evaluation/ball.h>

#include <g3log/g3log.hpp>

#include "ai/hl/stp/evaluation/ball.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/passing/pass_generator.h"
#include "shared/constants.h"
#include "util/logger/custom_logging_levels.h"

using namespace Passing;

const std::string FreeKickPlay::name = "Free Kick Play";

FreeKickPlay::FreeKickPlay() : MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(5)) {}

std::string FreeKickPlay::getName() const
{
    return FreeKickPlay::name;
}

bool FreeKickPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurFreeKick();
}

bool FreeKickPlay::invariantHolds(const World &world) const
{
    return !Evaluation::teamHasPossession(world.enemyTeam(), world.ball());
}

void FreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    /**
     * Plan:
     * The bot closest to the ball takes the free kick. 
     * If no shot on net, try to pass a la corner kick
     * Else, just chip towards the enemy goal???
     * 
     * Other bots should move to the other side of field to lure enemies.
     */


    // We want the two cherry pickers to be in rectangles on the +y and -y sides of the
    // field in the +x half. We also further offset the rectangle from the goal line
    // for the cherry-picker closer to where we're taking the corner kick from
    Vector pos_y_goalline_x_offset(world.field().enemyDefenseArea().width(), 0);
    Vector neg_y_goalline_x_offset(world.field().enemyDefenseArea().width(), 0);
    if (kick_from_pos_corner)
    {
        pos_y_goalline_x_offset += {world.field().enemyDefenseArea().width(), 0};
    }
    else
    {
        // kick from neg corner
        neg_y_goalline_x_offset += {world.field().enemyDefenseArea().width(), 0};
    }
    Vector center_line_x_offset(1, 0);
    Rectangle pos_y_cherry_pick_rectangle(
        world.field().centerPoint() + center_line_x_offset,
        world.field().enemyCornerPos() - pos_y_goalline_x_offset);
    Rectangle neg_y_cherry_pick_rectangle(
        world.field().centerPoint() + center_line_x_offset,
        world.field().enemyCornerNeg() - neg_y_goalline_x_offset);

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_pos_y =
        std::make_shared<CherryPickTactic>(world, pos_y_cherry_pick_rectangle);
    auto cherry_pick_tactic_neg_y =
        std::make_shared<CherryPickTactic>(world, neg_y_cherry_pick_rectangle);

    // Setup two bait robots on the opposite side of the field to where the corner kick
    // is taking place to pull enemies away from the goal
    Point opposite_corner_to_kick = kick_from_pos_corner ? world.field().enemyCornerNeg()
                                                         : world.field().enemyCornerPos();
    Point bait_move_tactic_1_pos =
        opposite_corner_to_kick - Vector(world.field().enemyDefenseArea().width() * 0.5,
                                         copysign(0.5, opposite_corner_to_kick.y()));
    Point bait_move_tactic_2_pos =
        opposite_corner_to_kick - Vector(world.field().enemyDefenseArea().width() * 1.5,
                                         copysign(0.5, opposite_corner_to_kick.y()));
    auto bait_move_tactic_1 = std::make_shared<MoveTactic>(true);
    auto bait_move_tactic_2 = std::make_shared<MoveTactic>(true);
    bait_move_tactic_1->updateParams(
        bait_move_tactic_1_pos,
        (world.field().enemyGoal() - bait_move_tactic_1_pos).orientation(), 0.0);
    bait_move_tactic_2->updateParams(
        bait_move_tactic_2_pos,
        (world.field().enemyGoal() - bait_move_tactic_2_pos).orientation(), 0.0);

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
    Vector ball_to_center_vec = Vector(0, 0) - world.ball().position();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateParams(
        world.ball().position() - ball_to_center_vec.norm(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_center_vec.orientation(), 0);
}

void FreeKickPlay::updatePassGenerator(PassGenerator &pass_generator)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}

// Register this play in the PlayFactory
static TPlayFactory<FreeKickPlay> factory;
