#include "software/sensor_fusion/sensor_fusion.h"

#include "software/logger/logger.h"

SensorFusion::SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config)
    : sensor_fusion_config(sensor_fusion_config),
      history_size(20),
      field(std::nullopt),
      ball(std::nullopt),
      friendly_team(),
      enemy_team(),
      game_state(),
      refbox_stage(std::nullopt),
      ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                  BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter(),
      enemy_team_filter(),
      ball_states(history_size)
{
    if (!sensor_fusion_config)
    {
        throw std::invalid_argument("SensorFusion created with null SensorFusionConfig");
    }
}

std::optional<World> SensorFusion::getWorld() const
{
    if (field && ball)
    {
        World new_world(*field, *ball, friendly_team, enemy_team);
        new_world.mutableGameState() = game_state;
        if (refbox_stage)
        {
            new_world.updateRefboxStage(*refbox_stage);
        }
        return new_world;
    }
    else
    {
        return std::nullopt;
    }
}

void SensorFusion::updateWorld(const SensorMsg &sensor_msg)
{
    if (sensor_msg.has_ssl_vision_msg())
    {
        updateWorld(sensor_msg.ssl_vision_msg());
    }

    if (sensor_msg.has_ssl_refbox_msg())
    {
        updateWorld(sensor_msg.ssl_refbox_msg());
    }

    updateWorld(sensor_msg.tbots_robot_msgs());
}

void SensorFusion::updateWorld(const SSL_WrapperPacket &packet)
{
    if (packet.has_geometry())
    {
        updateWorld(packet.geometry());
    }

    if (packet.has_detection())
    {
        updateWorld(packet.detection());
    }
}

void SensorFusion::updateWorld(const SSL_GeometryData &geometry_packet)
{
    field = createField(geometry_packet);
    if (!field)
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }
}

void SensorFusion::updateWorld(const SSL_Referee &packet)
{
    std::scoped_lock game_state_lock(game_state_mutex);
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    if (sensor_fusion_config->FriendlyColorYellow()->value())
    {
        game_state.updateRefboxGameState(
            createRefboxGameState(packet, TeamColour::YELLOW));
    }
    else
    {
        game_state.updateRefboxGameState(createRefboxGameState(packet, TeamColour::BLUE));
    }

    if (game_state.isOurBallPlacement())
    {
        auto pt = getBallPlacementPoint(packet);
        if (pt)
        {
            game_state.setBallPlacementPoint(*pt);
        }
        else
        {
            LOG(WARNING)
                << "In BALL_PLACEMENT_US game state, but no ball placement point found"
                << std::endl;
        }
    }

    refbox_stage = createRefboxStage(packet);
}

void SensorFusion::updateWorld(
    const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs)
{
    // TODO (issue #1149): incorporate TbotsRobotMsg into world and update world
}

void SensorFusion::updateWorld(const SSL_DetectionFrame &ssl_detection_frame)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    double min_valid_x = sensor_fusion_config->MinValidX()->value();
    double max_valid_x = sensor_fusion_config->MaxValidX()->value();
    bool ignore_invalid_camera_data =
        sensor_fusion_config->IgnoreInvalidCameraData()->value();

    // We invert the field side if we explicitly choose to override the values
    // provided by refbox. The 'defending_positive_side' parameter dictates the side
    // we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
//    const bool override_refbox_defending_side =
//        sensor_fusion_config->OverrideRefboxDefendingSide()->value();
//    const bool defending_positive_side =
//        sensor_fusion_config->DefendingPositiveSide()->value();
//    const bool should_invert_field =
//        override_refbox_defending_side && defending_positive_side;

    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    bool friendly_team_is_yellow = sensor_fusion_config->FriendlyColorYellow()->value();

    const bool should_invert_field = !friendly_team_is_yellow;

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
    if (friendly_team_is_yellow)
    {
        friendly_team = createFriendlyTeam(yellow_team);
        enemy_team    = createEnemyTeam(blue_team);
    }
    else
    {
        friendly_team = createFriendlyTeam(blue_team);
        enemy_team    = createEnemyTeam(yellow_team);
    }

    if (new_ball_state)
    {
        updateBall(*new_ball_state);
    }
}

void SensorFusion::updateBall(TimestampedBallState new_ball_state)
{
    std::scoped_lock game_state_lock(game_state_mutex);
    if (!ball_states.empty() &&
        new_ball_state.timestamp() < ball_states.front().timestamp())
    {
        throw std::invalid_argument(
            "Error: Trying to update ball state using a state older then the current state");
    }

    ball_states.push_front(new_ball_state);

    if (ball)
    {
        ball->updateState(new_ball_state);
    }
    else
    {
        ball = Ball(new_ball_state);
    }

    game_state.updateBall(*ball);
}

std::optional<TimestampedBallState> SensorFusion::createTimestampedBallState(
    const std::vector<BallDetection> &ball_detections)
{
    if (field)
    {
        std::optional<TimestampedBallState> new_ball =
            ball_filter.getFilteredData(ball_detections, *field);
        return new_ball;
    }
    return std::nullopt;
}

Team SensorFusion::createFriendlyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_friendly_team =
        friendly_team_filter.getFilteredData(friendly_team, robot_detections);
    RobotId friendly_goalie_id = sensor_fusion_config->FriendlyGoalieId()->value();
    new_friendly_team.assignGoalie(friendly_goalie_id);
    return new_friendly_team;
}

Team SensorFusion::createEnemyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_enemy_team = enemy_team_filter.getFilteredData(enemy_team, robot_detections);
    RobotId enemy_goalie_id = sensor_fusion_config->EnemyGoalieId()->value();
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
