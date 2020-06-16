#include "software/sensor_fusion/sensor_fusion.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"

SensorFusion::SensorFusion()
    : ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                  BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter(),
      enemy_team_filter()
{
}

void SensorFusion::onValueReceived(SensorMsg sensor_msg)
{
    updateWorld(sensor_msg);
    if (field && ball)
    {
        Subject<World>::sendValueToObservers(
            World(*field, *ball, friendly_team, enemy_team));
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

    updateWorld(sensor_msg.tbots_robot_msg());
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
        game_state = createRefboxGameState(packet, TeamColour::YELLOW);
    }
    else
    {
        game_state = createRefboxGameState(packet, TeamColour::BLUE);
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
    ;



    SSL_DetectionFrame detection_frame = ssl_detection_frame;
    // We invert the field side if we explicitly choose to override the values
    // provided by refbox. The 'defending_positive_side' parameter dictates the side
    // we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    if (Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->OverrideRefboxDefendingSide()
            ->value() &&
        Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->DefendingPositiveSide()
            ->value())
    {
        invertFieldSide(detection_frame);
    }

    std::optional<TimestampedBallState> new_ball_state;
    if (isCameraEnabled(detection_frame))
    {
        new_ball_state = createTimestampedBallState(createBallDetections(
            {detection_frame}, min_valid_x, max_valid_x, ignore_invalid_camera_data));

        auto yellow_team =
            createTeamDetection({detection_frame}, TeamColour::YELLOW, min_valid_x,
                                max_valid_x, ignore_invalid_camera_data);
        auto blue_team =
            createTeamDetection({detection_frame}, TeamColour::BLUE, min_valid_x,
                                max_valid_x, ignore_invalid_camera_data);

        // TODO remove Util::DynamicParameters as part of
        // https://github.com/UBC-Thunderbots/Software/issues/960
        if (Util::DynamicParameters->getAIControlConfig()
                ->getRefboxConfig()
                ->FriendlyColorYellow()
                ->value())
        {
            friendly_team = createFriendlyTeam(yellow_team);
            enemy_team    = createEnemyTeam(blue_team);
        }
        else
        {
            friendly_team = createFriendlyTeam(blue_team);
            enemy_team    = createEnemyTeam(yellow_team);
        }
    }

    if (new_ball_state)
    {
        if (ball)
        {
            ball->updateState(*new_ball_state);
        }
        else
        {
            ball = Ball(*new_ball_state);
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

void SensorFusion::invertFieldSide(SSL_DetectionFrame &frame)
{
    for (SSL_DetectionBall &ball : *frame.mutable_balls())
    {
        ball.set_x(-ball.x());
        ball.set_y(-ball.y());
    }
    for (const auto &team : {frame.mutable_robots_yellow(), frame.mutable_robots_blue()})
    {
        for (SSL_DetectionRobot &robot : *team)
        {
            robot.set_x(-robot.x());
            robot.set_y(-robot.y());
            robot.set_orientation(robot.orientation() + M_PI);
        }
    }
}

bool SensorFusion::isCameraEnabled(const SSL_DetectionFrame &detection)
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
            LOG(WARNING) << "An unkown camera id was detected, disabled by default "
                         << "id: " << detection.camera_id() << std::endl;
            camera_disabled = true;
            break;
    }
    return !camera_disabled;
}
