#include "proto/message_translation/tbots_protobuf.h"


std::unique_ptr<TbotsProto::Vision> createVision(const World& world)
{
    // create msg and set timestamp
    auto vision_msg                    = std::make_unique<TbotsProto::Vision>();
    *(vision_msg->mutable_time_sent()) = *createCurrentTimestamp();

    // set robot_states map
    auto& robot_states_map = *vision_msg->mutable_robot_states();
    auto friendly_robots   = world.friendlyTeam().getAllRobots();

    // For every friendly robot, we create a RobotState proto. The unique_ptr
    // is dereferenced, and there is an implicit deep copy into robot_states_map
    //
    // Since the unique_ptr immediately loses scope after the copy, the memory is
    // freed
    std::for_each(friendly_robots.begin(), friendly_robots.end(),
                  [&](const Robot& robot) {
                      robot_states_map[robot.id()] = *createRobotStateProto(robot);
                  });

    // set ball state
    *(vision_msg->mutable_ball_state()) = *createBallState(world.ball());

    return vision_msg;
}

std::unique_ptr<TbotsProto::World> createWorld(const World& world)
{
    // create msg
    auto world_msg                        = std::make_unique<TbotsProto::World>();
    *(world_msg->mutable_time_sent())     = *createCurrentTimestamp();
    *(world_msg->mutable_field())         = *createField(world.field());
    *(world_msg->mutable_friendly_team()) = *createTeam(world.friendlyTeam());
    *(world_msg->mutable_enemy_team())    = *createTeam(world.enemyTeam());
    *(world_msg->mutable_ball())          = *createBall(world.ball());
    *(world_msg->mutable_game_state())    = *createGameState(world.gameState());

    return world_msg;
}

std::unique_ptr<TbotsProto::Team> createTeam(const Team& team)
{
    // create msg
    auto team_msg      = std::make_unique<TbotsProto::Team>();
    const auto& robots = team.getAllRobots();

    std::for_each(robots.begin(), robots.end(), [&](const Robot& robot) {
        *(team_msg->add_team_robots()) = *createRobot(robot);
    });

    auto goalie_id = team.getGoalieId();
    if (goalie_id.has_value())
    {
        team_msg->set_goalie_id(goalie_id.value());
    }

    return team_msg;
}

std::unique_ptr<TbotsProto::Robot> createRobot(const Robot& robot)
{
    // create msg
    auto robot_msg = std::make_unique<TbotsProto::Robot>();
    robot_msg->set_id(robot.id());
    *(robot_msg->mutable_current_state()) = *createRobotStateProto(robot);
    *(robot_msg->mutable_timestamp())     = *createTimestamp(robot.timestamp());

    for (RobotCapability capability : robot.getUnavailableCapabilities())
    {
        switch (capability)
        {
            case RobotCapability::Dribble:
                robot_msg->add_unavailable_capabilities(
                    TbotsProto::Robot_RobotCapability_Dribble);
                break;
            case RobotCapability::Kick:
                robot_msg->add_unavailable_capabilities(
                    TbotsProto::Robot_RobotCapability_Kick);
                break;
            case RobotCapability::Chip:
                robot_msg->add_unavailable_capabilities(
                    TbotsProto::Robot_RobotCapability_Chip);
                break;
            case RobotCapability::Move:
                robot_msg->add_unavailable_capabilities(
                    TbotsProto::Robot_RobotCapability_Move);
                break;
        }
    }

    return robot_msg;
}

std::unique_ptr<TbotsProto::Ball> createBall(const Ball& ball)
{
    // create msg
    auto ball_msg                        = std::make_unique<TbotsProto::Ball>();
    *(ball_msg->mutable_current_state()) = *createBallState(ball);
    *(ball_msg->mutable_timestamp())     = *createTimestamp(ball.timestamp());

    return ball_msg;
}

std::unique_ptr<TbotsProto::Field> createField(const Field& field)
{
    // create msg
    auto field_msg = std::make_unique<TbotsProto::Field>();

    field_msg->set_field_x_length(field.xLength());
    field_msg->set_field_y_length(field.yLength());
    field_msg->set_defense_x_length(field.defenseAreaXLength());
    field_msg->set_defense_y_length(field.defenseAreaYLength());
    field_msg->set_goal_x_length(field.goalXLength());
    field_msg->set_goal_y_length(field.goalYLength());
    field_msg->set_boundary_buffer_size(field.boundaryMargin());
    field_msg->set_center_circle_radius(field.centerCircleRadius());

    return field_msg;
}

std::unique_ptr<TbotsProto::RobotState> createRobotStateProto(const Robot& robot)
{
    auto position         = createPointProto(robot.position());
    auto orientation      = createAngleProto(robot.orientation());
    auto velocity         = createVectorProto(robot.velocity());
    auto angular_velocity = createAngularVelocityProto(robot.angularVelocity());

    auto robot_state_msg = std::make_unique<TbotsProto::RobotState>();

    *(robot_state_msg->mutable_global_position())         = *position;
    *(robot_state_msg->mutable_global_orientation())      = *orientation;
    *(robot_state_msg->mutable_global_velocity())         = *velocity;
    *(robot_state_msg->mutable_global_angular_velocity()) = *angular_velocity;

    return robot_state_msg;
}

std::unique_ptr<TbotsProto::GameState> createGameState(const GameState& game_state)
{
    auto game_state_msg = std::make_unique<TbotsProto::GameState>();

    switch (game_state.getPlayState())
    {
        case GameState::HALT:
            game_state_msg->set_play_state(
                TbotsProto::GameState_PlayState_PLAY_STATE_HALT);
            break;
        case GameState::STOP:
            game_state_msg->set_play_state(
                TbotsProto::GameState_PlayState_PLAY_STATE_STOP);
            break;
        case GameState::SETUP:
            game_state_msg->set_play_state(
                TbotsProto::GameState_PlayState_PLAY_STATE_SETUP);
            break;
        case GameState::READY:
            game_state_msg->set_play_state(
                TbotsProto::GameState_PlayState_PLAY_STATE_READY);
            break;
        case GameState::PLAYING:
            game_state_msg->set_play_state(
                TbotsProto::GameState_PlayState_PLAY_STATE_PLAYING);
            break;
    }

    switch (game_state.getRestartReason())
    {
        case GameState::NONE:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_NONE);
            break;
        case GameState::KICKOFF:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_KICKOFF);
            break;
        case GameState::DIRECT:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_DIRECT);
            break;
        case GameState::INDIRECT:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_INDIRECT);
            break;
        case GameState::PENALTY:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_PENALTY);
            break;
        case GameState::BALL_PLACEMENT:
            game_state_msg->set_restart_reason(
                TbotsProto::GameState_RestartReason_RESTART_REASON_BALL_PLACEMENT);
            break;
    }

    switch (game_state.getRefereeCommand())
    {
        case RefereeCommand::HALT:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_HALT);
            break;
        case RefereeCommand::STOP:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_STOP);
            break;
        case RefereeCommand::NORMAL_START:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_NORMAL_START);
            break;
        case RefereeCommand::FORCE_START:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_FORCE_START);
            break;
        case RefereeCommand::PREPARE_KICKOFF_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_KICKOFF_US);
            break;
        case RefereeCommand::PREPARE_KICKOFF_THEM:
            game_state_msg->set_command(
                TbotsProto::
                    GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_KICKOFF_THEM);
            break;
        case RefereeCommand::PREPARE_PENALTY_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_PENALTY_US);
            break;
        case RefereeCommand::PREPARE_PENALTY_THEM:
            game_state_msg->set_command(
                TbotsProto::
                    GameState_RefereeCommand_REFEREE_COMMAND_PREPARE_PENALTY_THEM);
            break;
        case RefereeCommand::DIRECT_FREE_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_DIRECT_FREE_US);
            break;
        case RefereeCommand::DIRECT_FREE_THEM:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_DIRECT_FREE_THEM);
            break;
        case RefereeCommand::INDIRECT_FREE_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_INDIRECT_FREE_US);
            break;
        case RefereeCommand::INDIRECT_FREE_THEM:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_INDIRECT_FREE_THEM);
            break;
        case RefereeCommand::TIMEOUT_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_TIMEOUT_US);
            break;
        case RefereeCommand::TIMEOUT_THEM:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_TIMEOUT_THEM);
            break;
        case RefereeCommand::GOAL_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_GOAL_US);
            break;
        case RefereeCommand::GOAL_THEM:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_GOAL_THEM);
            break;
        case RefereeCommand::BALL_PLACEMENT_US:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_BALL_PLACEMENT_US);
            break;
        case RefereeCommand::BALL_PLACEMENT_THEM:
            game_state_msg->set_command(
                TbotsProto::GameState_RefereeCommand_REFEREE_COMMAND_BALL_PLACEMENT_THEM);
            break;
    }

    auto ball_state = game_state.getBall();
    if (ball_state.has_value())
    {
        *(game_state_msg->mutable_ball()) = *createBall(ball_state.value());
    }

    auto ball_placement_point = game_state.getBallPlacementPoint();
    if (ball_placement_point.has_value())
    {
        *(game_state_msg->mutable_ball_placement_point()) =
            *createPointProto(ball_placement_point.value());
    }

    return game_state_msg;
}

std::unique_ptr<TbotsProto::BallState> createBallState(const Ball& ball)
{
    auto position       = createPointProto(ball.position());
    auto velocity       = createVectorProto(ball.velocity());
    auto ball_state_msg = std::make_unique<TbotsProto::BallState>();

    *(ball_state_msg->mutable_global_position()) = *position;
    *(ball_state_msg->mutable_global_velocity()) = *velocity;
    ball_state_msg->set_distance_from_ground(ball.currentState().distanceFromGround());

    return ball_state_msg;
}

std::unique_ptr<TbotsProto::Timestamp> createTimestamp(const Timestamp& timestamp)
{
    auto timestamp_msg = std::make_unique<TbotsProto::Timestamp>();
    timestamp_msg->set_epoch_timestamp_seconds(timestamp.toSeconds());
    return timestamp_msg;
}

std::unique_ptr<TbotsProto::Timestamp> createCurrentTimestamp()
{
    auto timestamp_msg    = std::make_unique<TbotsProto::Timestamp>();
    const auto clock_time = std::chrono::system_clock::now();
    double time_in_seconds =
        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                clock_time.time_since_epoch())
                                .count()) /
        MICROSECONDS_PER_SECOND;

    timestamp_msg->set_epoch_timestamp_seconds(time_in_seconds);
    return timestamp_msg;
}

RobotState createRobotState(const TbotsProto::RobotState robot_state)
{
    return RobotState(createPoint(robot_state.global_position()),
                      createVector(robot_state.global_velocity()),
                      createAngle(robot_state.global_orientation()),
                      createAngularVelocity(robot_state.global_angular_velocity()));
}
