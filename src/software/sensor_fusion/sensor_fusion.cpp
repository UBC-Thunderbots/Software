#include "software/sensor_fusion/sensor_fusion.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"

SensorFusion::SensorFusion()
    : field_state(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0)),
      ball_state(Point(), Vector(), Timestamp::fromSeconds(0)),
      friendly_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      enemy_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                  BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter(),
      enemy_team_filter(),
      ssl_protobuf_reader()
{
}

void SensorFusion::onValueReceived(SensorMsg sensor_msg)
{
    updateWorld(sensor_msg);
    if (world.field().isValid())
    {
        Subject<World>::sendValueToObservers(world);
    }
}

void SensorFusion::updateWorld(SensorMsg sensor_msg)
{
    if (sensor_msg.has_ssl_vision_msg())
    {
        updateWorld(*sensor_msg.mutable_ssl_vision_msg());
    }

    if (sensor_msg.has_ssl_refbox_msg())
    {
        updateWorld(*sensor_msg.mutable_ssl_refbox_msg());
    }

    updateWorld(*sensor_msg.mutable_tbots_robot_msg());
}

void SensorFusion::updateWorld(SSL_WrapperPacket packet)
{
    if (packet.has_geometry())
    {
        const auto &latest_geometry_data = packet.geometry();
        Field field = ssl_protobuf_reader.getFieldData(latest_geometry_data);
        world.updateFieldGeometry(field);
    }

    if (packet.has_detection())
    {
        SSL_DetectionFrame detection = *packet.mutable_detection();
        VisionDetection vision_detection =
            ssl_protobuf_reader.getVisionDetection(detection);
        updateWorld(vision_detection);
    }
}

void SensorFusion::updateWorld(Referee packet)
{
    world.updateRefboxGameState(ssl_protobuf_reader.getRefboxGameState(packet));
    world.updateRefboxStage(ssl_protobuf_reader.getRefboxStage(packet));
}

void SensorFusion::updateWorld(RepeatedPtrField<TbotsRobotMsg> tbots_robot_msgs)
{
    // TODO: incorporate TbotsRobotMsg into world and update world
    // https://github.com/UBC-Thunderbots/Software/issues/1149
}

void SensorFusion::updateWorld(const VisionDetection &vision_detection)
{
    world.mutableEnemyTeam()    = getEnemyTeamFromVisionDetection(vision_detection);
    world.mutableFriendlyTeam() = getFriendlyTeamFromVisionDetection(vision_detection);

    std::optional<Ball> new_ball = getBallFromVisionDetection(vision_detection);
    if (new_ball)
    {
        world.updateBallState(new_ball->currentState());
    }
}

std::optional<Ball> SensorFusion::getBallFromVisionDetection(
    const VisionDetection &vision_detection)
{
    std::vector<BallDetection> ball_detections = vision_detection.getBallDetections();
    std::optional<Ball> new_ball =
        ball_filter.getFilteredData(ball_detections, field_state);
    return new_ball;
}

Team SensorFusion::getFriendlyTeamFromVisionDetection(
    const VisionDetection &vision_detection)
{
    std::vector<RobotDetection> friendly_robot_detections =
        vision_detection.getFriendlyTeamDetections();
    Team new_friendly_team = friendly_team_filter.getFilteredData(
        friendly_team_state, friendly_robot_detections);
    int friendly_goalie_id = Util::DynamicParameters->getAIControlConfig()
                                 ->getRefboxConfig()
                                 ->FriendlyGoalieId()
                                 ->value();
    new_friendly_team.assignGoalie(friendly_goalie_id);
    return new_friendly_team;
}

Team SensorFusion::getEnemyTeamFromVisionDetection(
    const VisionDetection &vision_detection)
{
    std::vector<RobotDetection> enemy_robot_detections =
        vision_detection.getEnemyTeamDetections();
    Team new_enemy_team =
        enemy_team_filter.getFilteredData(enemy_team_state, enemy_robot_detections);
    int enemy_goalie_id = Util::DynamicParameters->getAIControlConfig()
                              ->getRefboxConfig()
                              ->EnemyGoalieId()
                              ->value();
    new_enemy_team.assignGoalie(enemy_goalie_id);
    return new_enemy_team;
}
