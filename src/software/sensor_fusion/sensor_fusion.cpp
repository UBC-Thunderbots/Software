#include "software/sensor_fusion/sensor_fusion.h"

#include "software/logger/logger.h"

SensorFusion::SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config)
    : sensor_fusion_config_(sensor_fusion_config),
      history_size_(20),
      field_(std::nullopt),
      ball_(std::nullopt),
      friendly_team_(),
      enemy_team_(),
      game_state_(),
      referee_stage_(std::nullopt),
      ball_filter_(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                   BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter_(),
      enemy_team_filter_(),
      ball_states_(history_size_)
{
    if (!sensor_fusion_config_)
    {
        throw std::invalid_argument("SensorFusion created with null SensorFusionConfig");
    }
}

std::optional<World> SensorFusion::getWorld() const
{
    if (field_ && ball_)
    {
        World new_world(*field_, *ball_, friendly_team_, enemy_team_);
        new_world.updateGameState(game_state_);
        if (referee_stage_)
        {
            new_world.updateRefereeStage(*referee_stage_);
        }
        return new_world;
    }
    else
    {
        return std::nullopt;
    }
}

void SensorFusion::updateWorld(const SensorProto &sensor_msg)
{
    if (sensor_msg.has_ssl_vision_msg())
    {
        auto packet = sensor_msg.ssl_vision_msg();
        if (packet.has_geometry())
        {
            updateField(packet.geometry());
        }
        if (packet.has_detection())
        {
            updateBallAndTeams(packet.detection(), sensor_msg.robot_status_msgs());
        }
    }

    if (sensor_msg.has_ssl_referee_msg())
    {
        updateRefereeStageAndGameState(sensor_msg.ssl_referee_msg());
    }
}

void SensorFusion::updateField(const SSLProto::SSL_GeometryData &geometry_packet)
{
    field_ = createField(geometry_packet);
    if (!field_)
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }
}

void SensorFusion::updateRefereeStageAndGameState(const SSLProto::Referee &packet)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    if (sensor_fusion_config_->FriendlyColorYellow()->value())
    {
        game_state_.updateRefereeCommand(
            createRefereeCommand(packet, TeamColour::YELLOW));
    }
    else
    {
        game_state_.updateRefereeCommand(createRefereeCommand(packet, TeamColour::BLUE));
    }

    if (game_state_.isOurBallPlacement())
    {
        auto pt = getBallPlacementPoint(packet);
        if (pt)
        {
            game_state_.setBallPlacementPoint(*pt);
        }
        else
        {
            LOG(WARNING)
                << "In BALL_PLACEMENT_US game state, but no ball placement point found"
                << std::endl;
        }
    }

    referee_stage_ = createRefereeStage(packet);
}

std::vector<RobotId> SensorFusion::getRobotsWithBreakBeamTriggered(
    const google::protobuf::RepeatedPtrField<TbotsProto::RobotStatus> &robot_status_msgs)
{
    std::vector<RobotId> friendly_robots_with_breakbeam_triggered;
    for (const auto &robot_status_msg : robot_status_msgs)
    {
        if (robot_status_msg.has_break_beam_status() &&
            robot_status_msg.break_beam_status().ball_in_beam())
        {
            friendly_robots_with_breakbeam_triggered.push_back(
                robot_status_msg.robot_id());
        }
    }
    return friendly_robots_with_breakbeam_triggered;
}

void SensorFusion::updateBallAndTeams(
    const SSLProto::SSL_DetectionFrame &ssl_detection_frame,
    const google::protobuf::RepeatedPtrField<TbotsProto::RobotStatus> &robot_status_msgs)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    double min_valid_x = sensor_fusion_config_->MinValidX()->value();
    double max_valid_x = sensor_fusion_config_->MaxValidX()->value();
    bool ignore_invalid_camera_data =
        sensor_fusion_config_->IgnoreInvalidCameraData()->value();

    // We invert the field side if we explicitly choose to override the values
    // provided by the game controller. The 'defending_positive_side' parameter dictates
    // the side we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    const bool override_game_controller_defending_side =
        sensor_fusion_config_->OverrideGameControllerDefendingSide()->value();
    const bool defending_positive_side =
        sensor_fusion_config_->DefendingPositiveSide()->value();
    const bool should_invert_field =
        override_game_controller_defending_side && defending_positive_side;

    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    bool friendly_team_is_yellow = sensor_fusion_config_->FriendlyColorYellow()->value();

    std::optional<TimestampedBallState> new_ball_state;
    auto ball_detections = createBallDetections({ssl_detection_frame}, min_valid_x,
                                                max_valid_x, ignore_invalid_camera_data);
    auto yellow_team =
        createTeamDetection({ssl_detection_frame}, TeamColour::YELLOW, min_valid_x,
                            max_valid_x, ignore_invalid_camera_data);
    auto blue_team =
        createTeamDetection({ssl_detection_frame}, TeamColour::BLUE, min_valid_x,
                            max_valid_x, ignore_invalid_camera_data);

    if (should_invert_field)
    {
        for (auto &detection : ball_detections)
        {
            detection = invert(detection);
        }
        for (auto &detection : yellow_team)
        {
            detection = invert(detection);
        }
        for (auto &detection : blue_team)
        {
            detection = invert(detection);
        }
    }

    new_ball_state = createTimestampedBallState(ball_detections);

    if (new_ball_state)
    {
        updateBall(*new_ball_state);
    }

    if (friendly_team_is_yellow)
    {
        friendly_team_ = createFriendlyTeam(yellow_team);
        enemy_team_    = createEnemyTeam(blue_team);
    }
    else
    {
        friendly_team_ = createFriendlyTeam(blue_team);
        enemy_team_    = createEnemyTeam(yellow_team);
    }

    friendly_team_.updateBallPossession(getRobotWithPossession(
        *ball_, friendly_team_, getRobotsWithBreakBeamTriggered(robot_status_msgs)));
    enemy_team_.updateBallPossession(getRobotWithPossession(*ball_, enemy_team_));
}

void SensorFusion::updateBall(TimestampedBallState new_ball_state)
{
    if (!ball_states_.empty() &&
        new_ball_state.timestamp() < ball_states_.front().timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }

    ball_states_.push_front(new_ball_state);

    if (ball_)
    {
        ball_->updateState(new_ball_state);
    }
    else
    {
        ball_ = Ball(new_ball_state);
    }

    game_state_.updateBall(*ball_);
}

std::optional<TimestampedBallState> SensorFusion::createTimestampedBallState(
    const std::vector<BallDetection> &ball_detections)
{
    if (field_)
    {
        std::optional<TimestampedBallState> new_ball =
            ball_filter_.getFilteredData(ball_detections, *field_);
        return new_ball;
    }
    return std::nullopt;
}

Team SensorFusion::createFriendlyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_friendly_team =
        friendly_team_filter_.getFilteredData(friendly_team_, robot_detections);
    RobotId friendly_goalie_id = sensor_fusion_config_->FriendlyGoalieId()->value();
    new_friendly_team.assignGoalie(friendly_goalie_id);
    return new_friendly_team;
}

Team SensorFusion::createEnemyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_enemy_team =
        enemy_team_filter_.getFilteredData(enemy_team_, robot_detections);
    RobotId enemy_goalie_id = sensor_fusion_config_->EnemyGoalieId()->value();
    new_enemy_team.assignGoalie(enemy_goalie_id);
    return new_enemy_team;
}

RobotDetection SensorFusion::invert(RobotDetection robot_detection) const
{
    robot_detection.position =
        Point(-robot_detection.position.x(), -robot_detection.position.y());
    robot_detection.orientation = robot_detection.orientation + Angle::half();
    return robot_detection;
}

BallDetection SensorFusion::invert(BallDetection ball_detection) const
{
    ball_detection.position =
        Point(-ball_detection.position.x(), -ball_detection.position.y());
    return ball_detection;
}
