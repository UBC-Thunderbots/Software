#pragma once

#include "proto/message_translation/tbots_geometry.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/visualization.pb.h"
#include "proto/world.pb.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

/**
 * Returns a TbotsProto::World proto given a World.
 *
 * @param world The world msg to extract the TbotsProto::World from
 * @param sequence_number An optional sequence number for tracking the TbotsProto::World
 *
 * @return The unique_ptr to a TbotsProto::World proto containing the field, friendly
 * team, enemy team, ball, and the game state.
 */
std::unique_ptr<TbotsProto::World> createWorld(const World& world,
                                               const uint64_t sequence_number = 0);

/**
 * Returns a TbotsProto::Team proto given a Team.
 *
 * @param team The Team msg to extract the TbotsProto::Team from
 *
 * @return The unique_ptr to a TbotsProto::Team proto containing a list of robots and
 * goalie ID
 */
std::unique_ptr<TbotsProto::Team> createTeam(const Team& team);

/**
 * Returns a TbotsProto::Robot proto given a Robot.
 *
 * @param robot The Robot msg to extract the TbotsProto::Robot from
 *
 * @return The unique_ptr to a TbotsProto::Robot proto containing the robot ID and robot
 * state
 */
std::unique_ptr<TbotsProto::Robot> createRobot(const Robot& robot);

/**
 * Returns a TbotsProto::Ball proto given a Ball.
 *
 * @param ball The Ball msg to extract the TbotsProto::Ball from
 *
 * @return The unique_ptr to a TbotsProto::Ball proto containing the ball state and the
 * ball acceleration
 */
std::unique_ptr<TbotsProto::Ball> createBall(const Ball& ball);

/**
 * Returns a TbotsProto::Field proto given a Field.
 *
 * @param field The Field msg to extract the TbotsProto::Field from
 *
 * @return The unique_ptr to a TbotsProto::Field proto containing the Ball ID and Ball
 * state
 */
std::unique_ptr<TbotsProto::Field> createField(const Field& field);

/**
 * Returns (Robot, Game, Ball) State given a (Robot, Game, Ball)
 *
 * @param The (Robot, Game, Ball) to convert to State proto
 *
 * @return The unique_ptr to a (Robot, Game, Ball) State after conversion
 */
std::unique_ptr<TbotsProto::RobotState> createRobotStateProto(const Robot& robot);
std::unique_ptr<TbotsProto::RobotState> createRobotStateProto(
    const RobotState& robot_state);
std::unique_ptr<TbotsProto::GameState> createGameState(const GameState& game_state);
std::unique_ptr<TbotsProto::BallState> createBallState(const Ball& ball);

/**
 * Returns a TbotsProto::Timestamp proto given a timestamp.
 *
 * @param timestamp The Timestamp msg to extract the TbotsProto::Timestamp from
 *
 * @return The unique_ptr to a TbotsProto::Timestamp proto containing the timestamp with
 * the same time zone as the timestamp argument.
 */
std::unique_ptr<TbotsProto::Timestamp> createTimestamp(const Timestamp& timestamp);

/**
 * Returns a TbotsProto::NamedValue proto given a name and value.
 *
 * @param name The name of the value to plot
 * @param value The NamedValue msg to extract the TbotsProto::NamedValue from
 *
 * @return The unique_ptr to a TbotsProto::NamedValue proto containing data with
 *         specified name and value
 */
std::unique_ptr<TbotsProto::NamedValue> createNamedValue(const std::string name,
                                                         float value);

/**
 * Returns a timestamp msg with the time that this function was called
 *
 * @return The unique_ptr to a TbotsProto::Timestamp with the current UTC time
 */
std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp();

/**
 * Return RobotState given the TbotsProto::RobotState protobuf
 *
 * @param robot_state The RobotState proto to create a RobotState from
 * @return the RobotState
 */
RobotState createRobotState(const TbotsProto::RobotState robot_state);

/**
 * Return BallState given the TbotsProto::BallState protobuf
 *
 * @param robot_state The BallState proto to create a RobotState from
 * @return the BallState
 */
BallState createBallState(const TbotsProto::BallState ball_state);

/**
 * Returns a pass visualization given a vector of the best passes
 *
 * @param A vector of passes across their fields  with their ratings
 *
 * @return The unique_ptr to a PassVisualization proto
 */
std::unique_ptr<TbotsProto::PassVisualization> createPassVisualization(
    const std::vector<PassWithRating>& passes_with_rating);
