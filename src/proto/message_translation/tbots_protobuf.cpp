#include "proto/message_translation/tbots_protobuf.h"


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
    return createRobotStateProto(robot.currentState());
}

std::unique_ptr<TbotsProto::RobotState> createRobotStateProto(
    const RobotState& robot_state)
{
    auto position         = createPointProto(robot_state.position());
    auto orientation      = createAngleProto(robot_state.orientation());
    auto velocity         = createVectorProto(robot_state.velocity());
    auto angular_velocity = createAngularVelocityProto(robot_state.angularVelocity());

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

std::unique_ptr<TbotsProto::NamedValue> createNamedValue(const std::string name,
                                                         float value)
{
    auto named_value_msg = std::make_unique<TbotsProto::NamedValue>();
    named_value_msg->set_name(name);
    named_value_msg->set_value(value);
    return named_value_msg;
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

BallState createBallState(const TbotsProto::BallState ball_state)
{
    return BallState(createPoint(ball_state.global_position()),
                     createVector(ball_state.global_velocity()),
                     ball_state.distance_from_ground());
}

std::unique_ptr<TbotsProto::PassVisualization> createPassVisualization(
    const std::vector<PassWithRating>& passes_with_rating)
{
    auto pass_visualization_msg = std::make_unique<TbotsProto::PassVisualization>();

    for (const auto& pass_with_rating : passes_with_rating)
    {
        auto pass_msg = std::make_unique<TbotsProto::Pass>();
        *(pass_msg->mutable_passer_point()) =
            *createPointProto(pass_with_rating.pass.passerPoint());
        *(pass_msg->mutable_receiver_point()) =
            *createPointProto(pass_with_rating.pass.receiverPoint());
        pass_msg->set_pass_speed_m_per_s(pass_with_rating.pass.speed());

        auto pass_with_rating_msg = std::make_unique<TbotsProto::PassWithRating>();
        pass_with_rating_msg->set_rating(pass_with_rating.rating);
        *(pass_with_rating_msg->mutable_pass_()) = *pass_msg;

        *(pass_visualization_msg->add_best_passes()) = *pass_with_rating_msg;
    }
    return pass_visualization_msg;
}

std::unique_ptr<TbotsProto::WorldStateReceivedTrigger> createWorldStateReceivedTrigger(
    const bool world_state_received_trigger)
{
    auto world_state_received_trigger_msg =
        std::make_unique<TbotsProto::WorldStateReceivedTrigger>();
    world_state_received_trigger_msg->set_world_state_received(world_state_received_trigger);
    return world_state_received_trigger_msg;
}

std::unique_ptr<TbotsProto::CostVisualization> createCostVisualization(
    const std::vector<double>& costs, int num_rows, int num_cols)
{
    auto cost_visualization_msg = std::make_unique<TbotsProto::CostVisualization>();
    cost_visualization_msg->set_num_rows(num_rows);
    cost_visualization_msg->set_num_cols(num_cols);

    for (const auto& cost : costs)
    {
        cost_visualization_msg->add_cost(cost);
    }

    return cost_visualization_msg;
}
