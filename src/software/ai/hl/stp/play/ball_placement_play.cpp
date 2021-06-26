#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/geom/algorithms/intersects.h"

#define CIRCLE_SHIT_YIELD(tactic_vector) {\
            std::optional<Point> center;\
            if (world.friendlyTeam().getGoalieId().has_value())\
            {\
                std::optional<Robot> goalie = world.friendlyTeam().getRobotById(world.friendlyTeam().getGoalieId().value());\
                if (goalie.has_value())\
                {\
                    center = goalie->position();\
                }\
            }\
            angle += Angle::fromDegrees(0.1);\
            if (!center.has_value())\
            {\
                center = Point(-2, 0);\
            }\
            if (world.gameState().getBallPlacementPoint().has_value() && world.gameState().getBallPlacementPoint()->x() < -2)\
            {\
                center = Point(-2, 0);\
            }\
            TacticVector result1 = circleCenter(move_tactics, center.value(), angle, tactic_vector);\
            yield({result1});\
           }


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

    Angle angle = Angle::fromDegrees(0);

    std::optional<Robot> robot;
    if (world.gameState().getBallPlacementPoint().has_value())
    {
        robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
    }

    Circle inflated_ball = Circle(world.ball().position(), ROBOT_MAX_RADIUS_METERS * 4);
    Rectangle field_boundary = world.field().fieldBoundary();
    std::array<Segment, 4> field_boundary_segments = {
            field_boundary.getTop(),
            field_boundary.getBottom(),
            field_boundary.getRight(),
            field_boundary.getLeft(),
    };

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    int num_intersections = 0;
    double closest = 10000.0;
    int closest_intersecting_field_bound = -1;
    for (std::size_t i = 0; i < field_boundary_segments.size(); i++)
    {
        const Segment &field_bound = field_boundary_segments[i];
        if (intersects(field_bound, inflated_ball))
        {
            num_intersections++;
            if (distance(world.ball().position(), field_bound) < closest) {
                closest_intersecting_field_bound = static_cast<int>(i);
            }
        }
    }

    std::optional<Point> pull_point;
    Vector intersecting_dir = world.ball().position().toVector();

    //TODO please actually write this properly, this is so bad please don't look beyond this point
    if (num_intersections >= 1) {
        if (closest_intersecting_field_bound == 0) {
            intersecting_dir = Vector(0, 1);
        } else if (closest_intersecting_field_bound == 1) {
            intersecting_dir = Vector(0, -1);
        } else if (closest_intersecting_field_bound == 2) {
            intersecting_dir = Vector(1, 0);
        } else if (closest_intersecting_field_bound == 3) {
            intersecting_dir = Vector(-1, 0);
        }

        do
        {
            begin = std::chrono::steady_clock::now();
            if (robot.has_value()) {
                align_to_ball_tactic->updateRobot(robot.value());
                align_to_ball_tactic->updateControlParams(
                        world.ball().position() - intersecting_dir.normalize(ROBOT_MAX_RADIUS_METERS * 2),
                        intersecting_dir.orientation(), 0.0);


                LOG (DEBUG) << "aligning";
                CIRCLE_SHIT_YIELD({align_to_ball_tactic});
            } else {
                if (world.gameState().getBallPlacementPoint().has_value()) {
                    robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
                }
            }
            end = std::chrono::steady_clock::now();
        } while (!align_to_ball_tactic->done() && std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() < 2);

        Point ball_pull_position = world.ball().position() - intersecting_dir.normalize(ROBOT_MAX_RADIUS_METERS * 5);
        pull_point = ball_pull_position;
        do {
            if (robot.has_value()) {
                place_ball_tactic->updateRobot(robot.value());
                place_ball_tactic->updateControlParams(ball_pull_position,
                                                       intersecting_dir.orientation(), true);

                CIRCLE_SHIT_YIELD({place_ball_tactic});

                LOG (DEBUG) << "dribbling 1";
            } else {
                if (world.gameState().getBallPlacementPoint().has_value()) {
                    robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
                }
            }
        } while (!place_ball_tactic->done());

        Point move_away_point = world.ball().position() -
                                Vector::createFromAngle(robot->orientation()).normalize(
                                        ROBOT_MAX_RADIUS_METERS * 4);
        do
        {
            begin = std::chrono::steady_clock::now();
            if (robot.has_value())
            {
                move_away_tactic->updateRobot(robot.value());
                move_away_tactic->updateControlParams(move_away_point,
                                                      robot->orientation(), 0.0,
                                                      DribblerMode::OFF,
                                                      BallCollisionType::ALLOW,
                                                      {AutoChipOrKickMode::OFF, 0},
                                                      MaxAllowedSpeedMode::PHYSICAL_LIMIT);
                LOG (DEBUG) << "moving away 1";
                CIRCLE_SHIT_YIELD({move_away_tactic});
                end = std::chrono::steady_clock::now();
            }
        } while (!move_away_tactic->done() && std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() < 2);
    }

    //TODO DONT LOOK PLEASE
    if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 1) {
        do {
            if (robot.has_value()) {
                if (world.gameState().getBallPlacementPoint().has_value()) {
                    if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 0.1) {
                        LOG (DEBUG) << "DJOEIWAJW";

                        LOG (DEBUG) << "moving away 2";
                        move_away_tactic->updateRobot(robot.value());
                        move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);

                        CIRCLE_SHIT_YIELD({move_away_tactic});
                    } else if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) < 1) {
                        do {

                            LOG (DEBUG) << "dribbling 2";
                            place_ball_tactic->updateRobot(robot.value());
                            place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint().value(),
                                                                   std::nullopt, true);

                            CIRCLE_SHIT_YIELD({place_ball_tactic});
                        } while (!place_ball_tactic->done());


                        Point move_away_point = world.ball().position() -
                                                Vector::createFromAngle(robot->orientation()).normalize(
                                                        ROBOT_MAX_RADIUS_METERS * 4);
                        if (!(contains(world.field().fieldBoundary(), move_away_point) && !contains(world.field().fieldLines(), move_away_point))) {

                            begin = std::chrono::steady_clock::now();
                            do {
                                LOG (DEBUG) << "breaking away 1";
                                if (robot.has_value()) {
                                    move_away_tactic->updateRobot(robot.value());
                                    move_away_tactic->updateControlParams(move_away_point,
                                                                          robot->orientation(), 0.0,
                                                                          DribblerMode::OFF,
                                                                          BallCollisionType::ALLOW,
                                                                          {AutoChipOrKickMode::OFF, 0},
                                                                          MaxAllowedSpeedMode::PHYSICAL_LIMIT);

                                    CIRCLE_SHIT_YIELD({ move_away_tactic });
                                }
                                end = std::chrono::steady_clock::now();
                            } while (!move_away_tactic->done() && std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() < 2);
                        }

                        do
                        {
                            if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1)
                            {
                                break;
                            }
                            LOG (DEBUG) << "breaking away 2";
                            if (robot.has_value()) {
                                move_away_tactic->updateRobot(robot.value());
                                move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0,
                                                                      DribblerMode::OFF,
                                                                      BallCollisionType::AVOID,
                                                                      {AutoChipOrKickMode::OFF, 0},
                                                                      MaxAllowedSpeedMode::TIPTOE);

                                CIRCLE_SHIT_YIELD({move_away_tactic});
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
        if (pull_point.has_value() && distance(world.ball().position(), pull_point.value()) > 0.1)
        {
            do {
                if (robot.has_value()) {
                    place_ball_tactic->updateRobot(robot.value());
                    place_ball_tactic->updateControlParams(pull_point.value(),
                                                           intersecting_dir.orientation(), true);

                    LOG (DEBUG) << "dribbling 3";
                    CIRCLE_SHIT_YIELD({place_ball_tactic});
                } else {
                    if (world.gameState().getBallPlacementPoint().has_value()) {
                        robot = world.friendlyTeam().getNearestRobot(world.gameState().getBallPlacementPoint().value());
                    }
                }
            } while (!place_ball_tactic->done());
        } else {
            if (world.gameState().getBallPlacementPoint().has_value()) {
                auto move_receiver_tactic = std::make_shared<MoveTactic>(false);
                auto move_passer_tactic = std::make_shared<MoveTactic>(false);

                Pass pass = Pass(world.ball().position(), world.gameState().getBallPlacementPoint().value(),
                                 4.5);

                std::optional<Robot> receiver_robot = world.friendlyTeam().getNearestRobot(pass.receiverPoint());
                do {
                    if ((pull_point.has_value() && distance(world.ball().position(), pull_point.value()) > 0.1) || world.ball().velocity().length() > 1) {
                        break;
                    }
                    pass = Pass(world.ball().position(), world.gameState().getBallPlacementPoint().value(),
                                4.5);
                    TacticVector tactics = {};
                    if (robot.has_value()) {
                        move_passer_tactic->updateRobot(robot.value());
                        move_passer_tactic->updateControlParams(pass.passerPoint() -
                                                                Vector::createFromAngle(
                                                                        pass.passerOrientation()).normalize(
                                                                        ROBOT_MAX_RADIUS_METERS * 4),
                                                                pass.passerOrientation(), 0.0);
                        tactics.emplace_back(move_passer_tactic);
                    }
                    if (receiver_robot.has_value()) {
                        move_receiver_tactic->updateRobot(receiver_robot.value());
                        move_receiver_tactic->updateControlParams(pass.receiverPoint(), pass.receiverOrientation(),
                                                                  0.0);
                        tactics.emplace_back(move_receiver_tactic);
                    }

                    if (move_receiver_tactic->done()) {
                        LOG (DEBUG) << "receiver not done";
                    }
                    if (move_passer_tactic->done()) {
                        LOG (DEBUG) << "passer not done";
                    }

                    CIRCLE_SHIT_YIELD(tactics);
                } while (!move_receiver_tactic->done() || !move_passer_tactic->done());

                auto receiver_tactic = std::make_shared<ReceiverTactic>(pass);
                begin = std::chrono::steady_clock::now();
                do {
                    if (robot.has_value() && receiver_robot.has_value()) {
                        pass_ball_tactic->updateRobot(robot.value());
                        pass_ball_tactic->updateControlParams(pass.passerPoint(), pass.receiverPoint(),
                                                              pass.speed());
                        receiver_tactic->updateRobot(receiver_robot.value());
                        receiver_tactic->updateControlParams(pass, true);

                        LOG (DEBUG) << "stuck passing";
                        TacticVector tactics = {receiver_tactic, pass_ball_tactic};
                        CIRCLE_SHIT_YIELD(tactics);
                    } else {
                        break;
                    }
                    end = std::chrono::steady_clock::now();
                } while (!receiver_tactic->done() && std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() < 2);

                do {
                    if (robot.has_value()) {
                        place_ball_tactic->updateRobot(robot.value());
                        place_ball_tactic->updateControlParams(pass.receiverPoint(),
                                                               std::nullopt, true);


                        LOG (DEBUG) << "dribble 4";
                        CIRCLE_SHIT_YIELD({ place_ball_tactic });
                    } else {
                        if (world.gameState().getBallPlacementPoint().has_value()) {
                            robot = world.friendlyTeam().getNearestRobot(
                                    world.gameState().getBallPlacementPoint().value());
                        }
                    }
                } while (!place_ball_tactic->done());

                do {
                    if (distance(world.ball().position(), world.gameState().getBallPlacementPoint().value()) > 0.1) {
                        break;
                    }
                    if (robot.has_value()) {
                        move_away_tactic->updateRobot(robot.value());
                        move_away_tactic->updateControlParams(last_waiting_point, Angle::zero(), 0.0);

                        CIRCLE_SHIT_YIELD({ move_away_tactic });
                        LOG (DEBUG) << "breaking away 4";
                    }
                } while (!move_away_tactic->done());
            }
        }
    }
}

TacticVector BallPlacementPlay::circleCenter(std::vector<std::shared_ptr<MoveTactic>> move_tactics, const Point& rotating_point, Angle angle, TacticVector current) {

    for (size_t k = 0; k < move_tactics.size(); k++) {

            Vector vector = Vector::createFromAngle(angle);
            Vector rot = vector.rotate(Angle::fromDegrees(static_cast<double>(k) * 90));

            move_tactics[k]->updateControlParams(
                    (rotating_point + rot.normalize(0.6)),
                    Angle::zero(), 0);
    }

    current.insert(current.end(), move_tactics.begin(), move_tactics.end());
    return current;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, PlayConfig> factory;
