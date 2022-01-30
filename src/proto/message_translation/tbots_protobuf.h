#pragma once

#include "proto/message_translation/tbots_geometry.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/vision.pb.h"
#include "proto/world.pb.h"
#include "software/world/world.h"

/**
 * Returns a TbotsProto::Vision proto given a World.
 *
 * @param world The world msg to extract the TbotsProto::Vision from
 *
 * @return The unique_ptr to a TbotsProto::Vision proto containing the friendly team and
 * ball information
 */
std::unique_ptr<TbotsProto::Vision> createVision(const World& world);

/**
 * Returns a TbotsProto::World proto given a World.
 *
 * @param world The world msg to extract the TbotsProto::World from
 *
 * @return The unique_ptr to a TbotsProto::World proto containing the field, friendly
 * team, enemy team, ball, and the game state.
 */
std::unique_ptr<TbotsProto::World> createWorld(const World& world);

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
std::unique_ptr<TbotsProto::RobotState> createRobotState(const Robot& robot);
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
 * Returns a timestamp msg with the time that this function was called
 *
 * @return The unique_ptr to a TbotsProto::Timestamp with the current UTC time
 */
std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp();

/**
 * Create Robot State
 *
 * @param robot_state_proto the robot state proto
 *
 * @return the equivalent RobotState
 */
RobotState createRobotState(const TbotsProto::RobotState& robot_state_proto);

/**
 * Create Ball State
 *
 * @param ball_state_proto the ball state proto
 *
 * @return the equivalent BallState
 */
BallState createBallState(const TbotsProto::BallState& ball_state_proto);
