#include "software/sensor_fusion/sensor_fusion.h"

#include <algorithm>

#include "software/logger/logger.h"

SensorFusion::SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config)
    : sensor_fusion_config_(sensor_fusion_config),
      history_size_(20),
      field_(std::nullopt),
      ball_(std::nullopt),
      friendly_team_(),
      enemy_team_(),
      timestamped_possession_state_(),
      game_state_(),
      refbox_stage_(std::nullopt),
      ball_filter_(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                   BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter_(),
      enemy_team_filter_(),
      possession_filter_(),
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
        new_world.mutableGameState() = game_state_;
        if (refbox_stage_)
        {
            new_world.updateRefboxStage(*refbox_stage_);
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
        auto packet = sensor_msg.ssl_vision_msg();
        if (packet.has_geometry())
        {
            updateField(packet.geometry());
        }
        if (packet.has_detection())
        {
            updateBallAndTeams(packet.detection());
        }
    }

    if (sensor_msg.has_ssl_refbox_msg())
    {
        updateRefboxStageAndGameState(sensor_msg.ssl_refbox_msg());
    }

    if (ball_)
    {
        updatePossessionState(sensor_msg.tbots_robot_msgs());
    }
}

void SensorFusion::updateField(const SSL_GeometryData &geometry_packet)
{
    field_ = createField(geometry_packet);
    if (!field_)
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }
}

void SensorFusion::updateRefboxStageAndGameState(const SSL_Referee &packet)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    if (sensor_fusion_config_->FriendlyColorYellow()->value())
    {
        game_state_.updateRefboxGameState(
            createRefboxGameState(packet, TeamColour::YELLOW));
    }
    else
    {
        game_state_.updateRefboxGameState(
            createRefboxGameState(packet, TeamColour::BLUE));
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

    refbox_stage_ = createRefboxStage(packet);
}

void SensorFusion::updatePossessionState(
    const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs)
{
    if (ball_)
    {
        std::vector<RobotId> friendly_robots_with_breakbeam_triggered;
        for (const auto &tbots_robot_msg : tbots_robot_msgs)
        {
            if (tbots_robot_msg.has_break_beam_status())
            {
                friendly_robots_with_breakbeam_triggered.push_back(
                    tbots_robot_msg.robot_id());
            }
        }

        Timestamp most_recent_timestamp =
            std::max<Timestamp>({friendly_team_.getMostRecentTimestamp(),
                                 enemy_team_.getMostRecentTimestamp()});
        timestamped_possession_state_.updateState(
            possession_filter_.getRobotsWithPossession(
                friendly_robots_with_breakbeam_triggered, friendly_team_, enemy_team_,
                *ball_),
            most_recent_timestamp);
    }
}

void SensorFusion::updateBallAndTeams(const SSL_DetectionFrame &ssl_detection_frame)
{
    // TODO remove DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    double min_valid_x = sensor_fusion_config_->MinValidX()->value();
    double max_valid_x = sensor_fusion_config_->MaxValidX()->value();
    bool ignore_invalid_camera_data =
        sensor_fusion_config_->IgnoreInvalidCameraData()->value();

    // We invert the field side if we explicitly choose to override the values
    // provided by refbox. The 'defending_positive_side' parameter dictates the side
    // we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    const bool override_refbox_defending_side =
        sensor_fusion_config_->OverrideRefboxDefendingSide()->value();
    const bool defending_positive_side =
        sensor_fusion_config_->DefendingPositiveSide()->value();
    const bool should_invert_field =
        override_refbox_defending_side && defending_positive_side;

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
            invert(detection);
        }
        for (auto &detection : yellow_team)
        {
            invert(detection);
        }
        for (auto &detection : blue_team)
        {
            invert(detection);
        }
    }

    new_ball_state = createTimestampedBallState(ball_detections);
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

    if (new_ball_state)
    {
        updateBall(*new_ball_state);
    }
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
