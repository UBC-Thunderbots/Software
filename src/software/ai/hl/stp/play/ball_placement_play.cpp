#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

BallPlacementPlay::BallPlacementPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config)
{
}

bool BallPlacementPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurBallPlacement();
}

bool BallPlacementPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurBallPlacement();
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    auto place_ball_tactic = std::make_shared<DribbleTactic>();

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    // non goalie and non ball placing robots line up along a line just outside the
    // friendly defense area to wait for ball placement to finish
    Vector waiting_line_vector = world.field().friendlyDefenseArea().posXPosYCorner() -
                                 world.field().friendlyDefenseArea().posXNegYCorner();
    Point waiting_line_start_point =
        world.field().friendlyDefenseArea().posXNegYCorner() +
        Vector(ROBOT_MAX_RADIUS_METERS * 2, 0);
    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point waiting_destination =
            waiting_line_start_point +
            waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                          static_cast<double>(move_tactics.size() - 1));
        move_tactics.at(i)->updateControlParams(waiting_destination, Angle::zero(), 0.0);
    }

    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig());

    do
    {
        place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint(),
                                               std::nullopt, true);
        TacticVector result = {goalie_tactic, place_ball_tactic};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, PlayConfig> factory;
