#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

BallPlacementPlay::BallPlacementPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy)
{
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const WorldPtr &world_ptr)
{
    auto place_ball_tactic =
        std::make_shared<AssignedSkillTactic<DribbleSkillFSM>>(strategy);

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()};

    // non goalie and non ball placing robots line up along a line just outside the
    // friendly defense area to wait for ball placement to finish
    Vector waiting_line_vector =
        world_ptr->field().friendlyDefenseArea().posXPosYCorner() -
        world_ptr->field().friendlyDefenseArea().posXNegYCorner();
    Point waiting_line_start_point =
        world_ptr->field().friendlyDefenseArea().posXNegYCorner() +
        Vector(ROBOT_MAX_RADIUS_METERS * 3,
               0);  // Path planner can slow down when pathing through
                    // objects - buffer zone of radius x 3 should help
    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point waiting_destination =
            waiting_line_start_point +
            waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                          static_cast<double>(move_tactics.size() - 1));
        move_tactics.at(i)->updateControlParams(waiting_destination, Angle::zero(), 0.0);
    }

    do
    {
        place_ball_tactic->updateControlParams(
            world_ptr->gameState().getBallPlacementPoint(), std::nullopt, true);
        TacticVector result = {place_ball_tactic};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, std::shared_ptr<Strategy>>
    factory;
