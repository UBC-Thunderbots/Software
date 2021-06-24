#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/geom/algorithms/intersects.h"

//TODO honestly just rewrite this entire play
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
    auto align_to_ball_tactic = std::make_shared<MoveTactic>(false);
    auto move_away_tactic = std::make_shared<MoveTactic>(false);
    auto pass_ball_tactic = std::make_shared<KickTactic>(false);

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

    Circle inflated_ball = Circle(world.ball().position(), ROBOT_MAX_RADIUS_METERS * 2);
    Rectangle field_boundary = world.field().fieldBoundary();
    std::array<Segment, 4> field_boundary_segments = {
            field_boundary.getTop(),
            field_boundary.getBottom(),
            field_boundary.getRight(),
            field_boundary.getLeft(),
    };

    int num_intersections = 0;
    int intersecting_field_bound = -1;
    for (std::size_t i = 0; i < field_boundary_segments.size(); i++)
    {
        const Segment &field_bound = field_boundary_segments[i];
        if (intersects(field_bound, inflated_ball))
        {
            num_intersections++;
            intersecting_field_bound = static_cast<int>(i);
        }
    }

    //TODO please actually write this properly, this is so bad please don't look beyond this point
    if (num_intersections == 1) {
        Vector intersecting_dir = world.ball().position().toVector();
        if (intersecting_field_bound == 0) {
            intersecting_dir = Vector(0, 1);
        } else if (intersecting_field_bound == 1) {
            intersecting_dir = Vector(0, -1);
        } else if (intersecting_field_bound == 2) {
            intersecting_dir = Vector(1, 0);
        } else if (intersecting_field_bound == 3) {
            intersecting_dir = Vector(-1, 0);
        }

        do
        {
            if (robot.has_value()) {
                align_to_ball_tactic->updateRobot(robot.value());
                align_to_ball_tactic->updateControlParams(
                        world.ball().position() - intersecting_dir.normalize(ROBOT_MAX_RADIUS_METERS * 2),
                        intersecting_dir.orientation(), 0.0);
                TacticVector result = {align_to_ball_tactic};
                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                yield({result});
            } else {
                if (world.gameState().getBallPlacementPoint().has_value()) {
                    robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
                }
            }
        } while (!align_to_ball_tactic->done());

        Point ball_pull_position = world.ball().position() - intersecting_dir.normalize(ROBOT_MAX_RADIUS_METERS * 2);
        do {
            if (robot.has_value()) {
                place_ball_tactic->updateRobot(robot.value());
                place_ball_tactic->updateControlParams(ball_pull_position,
                                                       intersecting_dir.orientation(), true);
                TacticVector result = {place_ball_tactic};
                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                yield({result});
            } else {
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
    }

    //TODO DONT LOOK PLEASE
    if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 1) {
        do {
            if (robot.has_value()) {
                if (world.gameState().getBallPlacementPoint().has_value()) {
                    if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 0.1) {
                        move_away_tactic->updateRobot(robot.value());
                        move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);
                        TacticVector result = {move_away_tactic};
                        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                        yield({result});
                    } else if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 1) {
                        do {
                            place_ball_tactic->updateRobot(robot.value());
                            place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint().value(),
                                                                   std::nullopt, true);
                            TacticVector result = {place_ball_tactic};
                            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                            yield({result});
                        } while (!place_ball_tactic->done());

                        do {
                            if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
                            {
                                break;
                            }
                            if (robot.has_value()) {
                                move_away_tactic->updateRobot(robot.value());
                                move_away_tactic->updateControlParams(
                                        world.ball().position() -
                                        Vector::createFromAngle(robot->orientation()).normalize(
                                                ROBOT_MAX_RADIUS_METERS * 2.5), robot->orientation(), 0.0);
                                TacticVector result = {move_away_tactic};
                                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                                yield({result});
                            }
                        } while (!move_away_tactic->done());

                        do
                        {
                            if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
                            {
                                break;
                            }
                            if (robot.has_value()) {
                                move_away_tactic->updateRobot(robot.value());
                                move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);
                                TacticVector result = {move_away_tactic};
                                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                                yield({result});
                            }
                        } while (!move_away_tactic->done());

                        //if it works it works ;)
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        } while (!place_ball_tactic->done() && !move_away_tactic->done());
    }
    else {

        if (world.gameState().getBallPlacementPoint().has_value()) {
            auto move_receiver_tactic = std::make_shared<MoveTactic>(false);
            auto move_passer_tactic = std::make_shared<MoveTactic>(false);

            //TODO change so we dont use goalie tactic config LOL
            Pass pass = Pass(world.ball().position(), world.gameState().getBallPlacementPoint().value(),
                             play_config->getGoalieTacticConfig()->getBlockConeRadius()->value() * 10);

            std::optional<Robot> receiver_robot = world.friendlyTeam().getNearestRobot(pass.receiverPoint());
            do {
                //TODO change so we dont use goalie tactic config LOL
                pass = Pass(world.ball().position(), world.gameState().getBallPlacementPoint().value(),
                            play_config->getGoalieTacticConfig()->getBlockConeRadius()->value() * 10);
                TacticVector result = {};
                if (robot.has_value()) {
                    move_passer_tactic->updateRobot(robot.value());
                    move_passer_tactic->updateControlParams(pass.passerPoint() -
                                                            Vector::createFromAngle(pass.passerOrientation()).normalize(
                                                                    ROBOT_MAX_RADIUS_METERS * 2.5),
                                                            pass.passerOrientation(), 0.0);
                    result.emplace_back(move_passer_tactic);
                }
                if (receiver_robot.has_value()) {
                    move_receiver_tactic->updateRobot(receiver_robot.value());
                    move_receiver_tactic->updateControlParams(pass.receiverPoint(), pass.receiverOrientation(), 0.0);
                    result.emplace_back(move_receiver_tactic);
                }

                result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                yield({result});
            } while (!move_receiver_tactic->done() || !move_passer_tactic->done());

            auto receiver_tactic = std::make_shared<ReceiverTactic>(pass);
            do {
                if (robot.has_value() && receiver_robot.has_value()) {
                    pass_ball_tactic->updateRobot(robot.value());
                    pass_ball_tactic->updateControlParams(pass.passerPoint(), pass.receiverPoint(),
                                                          pass.speed());
                    receiver_tactic->updateRobot(receiver_robot.value());
                    receiver_tactic->updateControlParams(pass, true);
                    TacticVector result = {receiver_tactic, pass_ball_tactic};
                    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                    yield({result});
                } else {
                    break;
                }
            } while (!receiver_tactic->done());

            do {
                if (robot.has_value()) {
                    place_ball_tactic->updateRobot(robot.value());
                    place_ball_tactic->updateControlParams(pass.receiverPoint(),
                                                           std::nullopt, true);
                    TacticVector result = {place_ball_tactic};
                    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                    yield({result});
                } else {
                    if (world.gameState().getBallPlacementPoint().has_value()) {
                        robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
                    }
                }
            } while (!place_ball_tactic->done());

            do {
                if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
                {
                    break;
                }
                if (receiver_robot.has_value()) {
                    move_away_tactic->updateRobot(receiver_robot.value());
                    move_away_tactic->updateControlParams(
                            world.ball().position() -
                            Vector::createFromAngle(receiver_robot->orientation()).normalize(
                                    ROBOT_MAX_RADIUS_METERS * 2.5), receiver_robot->orientation(), 0.0);
                    TacticVector result = {move_away_tactic};
                    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                    yield({result});
                }
            } while (!move_away_tactic->done());

            do
            {
                if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
                {
                    break;
                }
                if (robot.has_value()) {
                    move_away_tactic->updateRobot(robot.value());
                    move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);
                    TacticVector result = {move_away_tactic};
                    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
                    yield({result});
                }
            } while (!move_away_tactic->done());
        }
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, PlayConfig> factory;
