#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

BallPlacementPlay::BallPlacementPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
{
}

bool BallPlacementPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurBallPlacement() && world.gameState().getBallPlacementPoint().has_value() && distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1;
}

bool BallPlacementPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurBallPlacement();
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    auto place_ball_tactic = std::make_shared<DribbleTactic>();
    auto move_away_tactic = std::make_shared<MoveTactic>(false);

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

    Point last_waiting_point = waiting_line_start_point + waiting_line_vector.normalize(waiting_line_vector.length() * static_cast<int>(move_tactics.size()) / static_cast<double>(move_tactics.size()));
    for (unsigned int i = 0; i < move_tactics.size(); i++) {
        Point waiting_destination =
                waiting_line_start_point +
                waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                              static_cast<double>(move_tactics.size()));
        move_tactics.at(i)->updateControlParams(waiting_destination, Angle::zero(), 0.0);
    }

    std::optional<Robot> robot;
    if (world.gameState().getBallPlacementPoint().has_value())
    {
        robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
    }

    do
    {
        if (robot.has_value()) {
            place_ball_tactic->updateRobot(robot.value());
            place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint(),
                                                     std::nullopt, true);
            TacticVector result = {place_ball_tactic};
            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
            yield({result});
        } else
        {
            if (world.gameState().getBallPlacementPoint().has_value()) {
                robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
            }
        }
    } while (!place_ball_tactic->done());

    do
    {
        if (robot.has_value())
        {
            move_away_tactic->updateRobot(robot.value());
            move_away_tactic->updateControlParams(
                    world.ball().position() -
                    Vector::createFromAngle(robot->orientation()).normalize(ROBOT_MAX_RADIUS_METERS * 2.5), robot->orientation(), 0.0);
            TacticVector result = {move_away_tactic};
            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
            yield({result});
        }
    } while (!move_away_tactic->done());

    do {
        if (robot.has_value()) {
            if (world.gameState().getBallPlacementPoint().has_value() && distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
            {
                TacticVector result = {place_ball_tactic};
                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                yield({result});
                continue;
            }

            move_away_tactic->updateRobot(robot.value());
            move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);
        }
        TacticVector result = {move_away_tactic};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (!move_away_tactic->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, PlayConfig> factory;
