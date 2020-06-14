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
    game_state   = createRefboxGameState(packet);
    refbox_stage = createRefboxStage(packet);
}

void SensorFusion::updateWorld(
    const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs)
{
    // TODO (issue #1149): incorporate TbotsRobotMsg into world and update world
}

void SensorFusion::updateWorld(const SSL_DetectionFrame &ssl_detection_frame)
{
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
        new_ball_state =
            createTimestampedBallState(createBallDetections({detection_frame}));
        friendly_team = createFriendlyTeam(
            createTeamDetection({detection_frame}, TeamType::FRIENDLY));
        enemy_team =
            createEnemyTeam(createTeamDetection({detection_frame}, TeamType::ENEMY));
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
