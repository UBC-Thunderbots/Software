#include "software/sensor_fusion/sensor_fusion.h"

#include "software/logger/logger.h"

SensorFusion::SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config)
    : sensor_fusion_config(sensor_fusion_config),
      field(std::nullopt),
      ball(std::nullopt),
      friendly_team(),
      enemy_team(),
      game_state(),
      referee_stage(std::nullopt),
      ball_filter(),
      friendly_team_filter(),
      enemy_team_filter(),
      team_with_possession(TeamSide::ENEMY)
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
        new_world.updateGameState(game_state);
        new_world.setTeamWithPossession(team_with_possession);
        if (referee_stage)
        {
            new_world.updateRefereeStage(*referee_stage);
        }
        return new_world;
    }
    else
    {
        return std::nullopt;
    }
}

void SensorFusion::processSensorProto(const SensorProto &sensor_msg)
{
    if (sensor_msg.has_ssl_vision_msg())
    {
        updateWorld(sensor_msg.ssl_vision_msg());
    }

    if (sensor_msg.has_ssl_referee_msg())
    {
        updateWorld(sensor_msg.ssl_referee_msg());
    }

    updateWorld(sensor_msg.robot_status_msgs());
}

void SensorFusion::updateWorld(const SSLProto::SSL_WrapperPacket &packet)
{
    if (packet.has_geometry())
    {
        updateWorld(packet.geometry());
    }

    if (packet.has_detection())
    {
        checkForVisionReset(packet.detection().t_capture());
        updateWorld(packet.detection());
    }
}

void SensorFusion::updateWorld(const SSLProto::SSL_GeometryData &geometry_packet)
{
    field = createField(geometry_packet);
    if (!field)
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }
}

void SensorFusion::updateWorld(const SSLProto::Referee &packet)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    if (sensor_fusion_config->FriendlyColorYellow()->value())
    {
        game_state.updateRefereeCommand(createRefereeCommand(packet, TeamColour::YELLOW));
    }
    else
    {
        game_state.updateRefereeCommand(createRefereeCommand(packet, TeamColour::BLUE));
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

    referee_stage = createRefereeStage(packet);
}

void SensorFusion::updateWorld(
    const google::protobuf::RepeatedPtrField<TbotsProto::RobotStatus> &robot_status_msgs)
{
    // TODO (#1819): Update robot capabilities based on robot status msgs
}

void SensorFusion::updateWorld(const SSLProto::SSL_DetectionFrame &ssl_detection_frame)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    double min_valid_x = sensor_fusion_config->MinValidX()->value();
    double max_valid_x = sensor_fusion_config->MaxValidX()->value();
    bool ignore_invalid_camera_data =
        sensor_fusion_config->IgnoreInvalidCameraData()->value();

    // We invert the field side if we explicitly choose to override the values
    // provided by the game controller. The 'defending_positive_side' parameter dictates
    // the side we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    const bool override_game_controller_defending_side =
        sensor_fusion_config->OverrideGameControllerDefendingSide()->value();
    const bool defending_positive_side =
        sensor_fusion_config->DefendingPositiveSide()->value();
    const bool should_invert_field =
        override_game_controller_defending_side && defending_positive_side;

    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    bool friendly_team_is_yellow = sensor_fusion_config->FriendlyColorYellow()->value();

    std::optional<Ball> new_ball;
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

    new_ball = createBall(ball_detections);
    if (new_ball)
    {
        updateBall(*new_ball);
    }

    if (ball)
    {
        bool friendly_team_has_ball = teamHasBall(friendly_team, *ball);
        bool enemy_team_has_ball    = teamHasBall(enemy_team, *ball);

        if (friendly_team_has_ball && !enemy_team_has_ball)
        {
            // take defensive view of exclusive possession for friendly possession
            team_with_possession = TeamSide::FRIENDLY;
        }

        if (enemy_team_has_ball)
        {
            team_with_possession = TeamSide::ENEMY;
        }
    }
}

void SensorFusion::updateBall(Ball new_ball)
{
    ball = new_ball;
    game_state.updateBall(*ball);
}

std::optional<Ball> SensorFusion::createBall(
    const std::vector<BallDetection> &ball_detections)
{
    if (field)
    {
        std::optional<Ball> new_ball =
            ball_filter.estimateBallState(ball_detections, field.value().fieldBoundary());
        return new_ball;
    }
    return std::nullopt;
}


Team SensorFusion::createFriendlyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_friendly_team =
        friendly_team_filter.getFilteredData(friendly_team, robot_detections);
    RobotId friendly_goalie_id = sensor_fusion_config->FriendlyGoalieId()->value();
    // TODO (1610) Implement friendly goalie ID override
    new_friendly_team.assignGoalie(friendly_goalie_id);
    return new_friendly_team;
}

Team SensorFusion::createEnemyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_enemy_team = enemy_team_filter.getFilteredData(enemy_team, robot_detections);
    RobotId enemy_goalie_id = sensor_fusion_config->EnemyGoalieId()->value();
    // TODO (1610) Implement enemy goalie ID override
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

bool SensorFusion::teamHasBall(const Team &team, const Ball &ball)
{
    for (const auto &robot : team.getAllRobots())
    {
        if (robot.isNearDribbler(ball.position()))
        {
            return true;
        }
    }
    return false;
}

void SensorFusion::checkForVisionReset(double t_capture)
{
    static unsigned int reset_time_vision_packets_detected = 0;
    static double last_t_capture                           = 0;

    if (t_capture < last_t_capture && t_capture < VISION_PACKET_RESET_TIME_THRESHOLD)
    {
        reset_time_vision_packets_detected++;
    }
    else
    {
        reset_time_vision_packets_detected = 0;
        last_t_capture                     = t_capture;
    }

    if (reset_time_vision_packets_detected > VISION_PACKET_RESET_COUNT_THRESHOLD)
    {
        LOG(WARNING) << "Vision reset detected... Resetting SensorFusion!" << std::endl;
        resetWorldComponents();
        last_t_capture                     = 0;
        reset_time_vision_packets_detected = 0;
    }
}

void SensorFusion::resetWorldComponents()
{
    field                = std::nullopt;
    ball                 = std::nullopt;
    friendly_team        = Team();
    enemy_team           = Team();
    game_state           = GameState();
    referee_stage        = std::nullopt;
    ball_filter          = BallFilter();
    friendly_team_filter = RobotTeamFilter();
    enemy_team_filter    = RobotTeamFilter();
    team_with_possession = TeamSide::ENEMY;
}
