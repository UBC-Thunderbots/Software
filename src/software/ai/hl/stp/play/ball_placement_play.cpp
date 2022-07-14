#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

BallPlacementPlay::BallPlacementPlay(TbotsProto::AiConfig config) : Play(config, true) {}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    auto place_ball_tactic = std::make_shared<DribbleTactic>(ai_config);
    auto move_away         = std::make_shared<MoveTactic>();
    auto stop              = std::make_shared<StopTactic>(false);

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()};

    // non goalie and non ball placing robots line up along a line just outside the
    // friendly defense area to wait for ball placement to finish
    Vector waiting_line_vector = world.field().friendlyDefenseArea().posXPosYCorner() -
                                 world.field().friendlyDefenseArea().posXNegYCorner();
    Point waiting_line_start_point =
        world.field().friendlyDefenseArea().posXNegYCorner() +
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
        place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint(),
                                               std::nullopt, true);
        TacticVector result = {place_ball_tactic};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (!place_ball_tactic->done());

    static int wait_counter = 0;

    while (wait_counter++ < 150)
    {
        TacticVector result = {stop};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    };

    do
    {
        move_away->updateControlParams(Point(-3, 0), Angle::zero(), 0.0);
        TacticVector result = {move_away};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, TbotsProto::AiConfig>
    factory;
