#include "software/sensor_fusion/sensor_fusion.h"

#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"

SensorFusion::SensorFusion()
    : history_size(20),
      field(std::nullopt),
      ball(std::nullopt),
      friendly_team(),
      enemy_team(),
      refbox_game_state(RefboxGameState::HALT),
      refbox_stage(std::nullopt),
      ball_placement_point(std::nullopt),
      ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                  BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter(),
      enemy_team_filter(),
      ball_states(history_size),
      friendly_robot_states_map(),
      enemy_robot_states_map()
{
}

std::optional<World> SensorFusion::getWorld() const
{
    if (field && ball)
    {
        World new_world(*field, *ball, friendly_team, enemy_team);
        if (refbox_stage)
        {
            new_world.updateRefboxStage(*refbox_stage);
        }
        if (ball_placement_point)
        {
            new_world.updateGameState(refbox_game_state, *ball_placement_point);
        }
        else
        {
            new_world.updateGameState(refbox_game_state);
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

void SensorFusion::updateWorld(const Referee &packet)
{
    // TODO remove Util::DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    if (Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->FriendlyColorYellow()
            ->value())
    {
        refbox_game_state = createRefboxGameState(packet, TeamColour::YELLOW);
    }
    else
    {
        refbox_game_state = createRefboxGameState(packet, TeamColour::BLUE);
    }

    if (refbox_game_state == RefboxGameState::BALL_PLACEMENT_US)
    {
        auto pt = getBallPlacementPoint(packet);
        if (pt)
        {
            ball_placement_point = pt;
        }
        else
        {
            LOG(WARNING)
                << "In BALL_PLACEMENT_US game state, but no ball placement point found"
                << std::endl;
        }
    }
    else
    {
        ball_placement_point = std::nullopt;
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
    // TODO remove Util::DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    double min_valid_x = Util::DynamicParameters->getAIControlConfig()
                             ->getRefboxConfig()
                             ->MinValidX()
                             ->value();
    double max_valid_x = Util::DynamicParameters->getAIControlConfig()
                             ->getRefboxConfig()
                             ->MaxValidX()
                             ->value();
    bool ignore_invalid_camera_data = Util::DynamicParameters->getAIControlConfig()
                                          ->getRefboxConfig()
                                          ->IgnoreInvalidCameraData()
                                          ->value();

    // We invert the field side if we explicitly choose to override the values
    // provided by refbox. The 'defending_positive_side' parameter dictates the side
    // we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    bool should_invert_field = Util::DynamicParameters->getAIControlConfig()
                                   ->getRefboxConfig()
                                   ->OverrideRefboxDefendingSide()
                                   ->value() &&
                               Util::DynamicParameters->getAIControlConfig()
                                   ->getRefboxConfig()
                                   ->DefendingPositiveSide()
                                   ->value();

    // TODO remove Util::DynamicParameters as part of
    // https://github.com/UBC-Thunderbots/Software/issues/960
    bool friendly_team_is_yellow = Util::DynamicParameters->getAIControlConfig()
                                       ->getRefboxConfig()
                                       ->FriendlyColorYellow()
                                       ->value();

    std::optional<TimestampedBallState> new_ball_state;
    if (isCameraEnabled(ssl_detection_frame))
    {
        auto ball_detections = createBallDetections(
            {ssl_detection_frame}, min_valid_x, max_valid_x, ignore_invalid_camera_data);
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
            friendly_team = createFriendlyTeam(yellow_team);
            enemy_team    = createEnemyTeam(blue_team);
        }
        else
        {
            friendly_team = createFriendlyTeam(blue_team);
            enemy_team    = createEnemyTeam(yellow_team);
        }
        updateRobotStatesMap();
    }

    if (new_ball_state)
    {
        updateBall(*new_ball_state);
    }
}

void SensorFusion::updateBall(TimestampedBallState new_ball_state)
{
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
}

void SensorFusion::updateRobotStatesMap()
{
    for (const auto &robot : friendly_team.getAllRobots())
    {
        auto it = friendly_robot_states_map.find(robot.id());
        if (it != friendly_robot_states_map.end())
        {
            if (!it->second.empty() &&
                robot.currentState().timestamp() < it->second.front().timestamp())
            {
                throw std::invalid_argument(
                    "Error: Trying to update robot state using a state older then the current state");
            }

            it->second.push_front(robot.currentState());
        }
        else
        {
            friendly_robot_states_map[robot.id()] =
                boost::circular_buffer<TimestampedRobotState>(history_size);
            friendly_robot_states_map[robot.id()].push_front(robot.currentState());
        }
    }

    for (const auto &robot : enemy_team.getAllRobots())
    {
        auto it = enemy_robot_states_map.find(robot.id());
        if (it != enemy_robot_states_map.end())
        {
            if (!it->second.empty() &&
                robot.currentState().timestamp() < it->second.front().timestamp())
            {
                throw std::invalid_argument(
                    "Error: Trying to update robot state using a state older then the current state");
            }

            it->second.push_front(robot.currentState());
        }
        else
        {
            enemy_robot_states_map[robot.id()] =
                boost::circular_buffer<TimestampedRobotState>(history_size);
            enemy_robot_states_map[robot.id()].push_front(robot.currentState());
        }
    }
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
    RobotId friendly_goalie_id = Util::DynamicParameters->getAIControlConfig()
                                     ->getRefboxConfig()
                                     ->FriendlyGoalieId()
                                     ->value();
    new_friendly_team.assignGoalie(friendly_goalie_id);
    return new_friendly_team;
}

Team SensorFusion::createEnemyTeam(const std::vector<RobotDetection> &robot_detections)
{
    Team new_enemy_team = enemy_team_filter.getFilteredData(enemy_team, robot_detections);
    RobotId enemy_goalie_id = Util::DynamicParameters->getAIControlConfig()
                                  ->getRefboxConfig()
                                  ->EnemyGoalieId()
                                  ->value();
    new_enemy_team.assignGoalie(enemy_goalie_id);
    return new_enemy_team;
}

bool SensorFusion::isCameraEnabled(const SSL_DetectionFrame &detection) const
{
    bool camera_disabled = false;
    switch (detection.camera_id())
    {
        // TODO: create an array of dynamic params to index into with camera_id()
        // may be resolved by https://github.com/UBC-Thunderbots/Software/issues/960
        case 0:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_0()->value();
            break;
        case 1:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_1()->value();
            break;
        case 2:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_2()->value();
            break;
        case 3:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_3()->value();
            break;
        default:
            LOG(WARNING) << "An unknown camera id was detected, disabled by default "
                         << "id: " << detection.camera_id() << std::endl;
            camera_disabled = true;
            break;
    }
    return !camera_disabled;
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
