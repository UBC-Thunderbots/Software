#include "software/sensor_fusion/sensor_fusion.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"

SensorFusion::SensorFusion()
    : ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
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
    std::optional<Field> field = ssl_protobuf_reader.getField(geometry_packet);
    if (field)
    {
        world.updateFieldGeometry(*field);
    }
    else
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }
}

void SensorFusion::updateWorld(const Referee &packet)
{
    world.updateRefboxGameState(ssl_protobuf_reader.getRefboxGameState(packet));
    world.updateRefboxStage(ssl_protobuf_reader.getRefboxStage(packet));
}

void SensorFusion::updateWorld(
    const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs)
{
    // TODO (issue #1149): incorporate TbotsRobotMsg into world and update world
}

void SensorFusion::updateWorld(const SSL_DetectionFrame &ssl_detection_frame)
{
    VisionDetection vision_detection =
        ssl_protobuf_reader.getVisionDetection(ssl_detection_frame);
    world.mutableEnemyTeam()     = getEnemyTeamFromVisionDetection(vision_detection);
    world.mutableFriendlyTeam()  = getFriendlyTeamFromVisionDetection(vision_detection);
    std::optional<Ball> new_ball = getBallFromVisionDetection(vision_detection);
    if (new_ball)
    {
        world.updateBallStateWithTimestamp(new_ball->currentState());
    }
}

std::optional<Ball> SensorFusion::getBallFromVisionDetection(
    const VisionDetection &vision_detection)
{
    std::vector<BallDetection> ball_detections = vision_detection.getBallDetections();
    std::optional<Ball> new_ball =
        ball_filter.getFilteredData(ball_detections, world.field());
    return new_ball;
}

Team SensorFusion::getFriendlyTeamFromVisionDetection(
    const VisionDetection &vision_detection)
{
    std::vector<RobotDetection> friendly_robot_detections =
        vision_detection.getFriendlyTeamDetections();
    Team new_friendly_team = friendly_team_filter.getFilteredData(
        world.friendlyTeam(), friendly_robot_detections);
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
        enemy_team_filter.getFilteredData(world.enemyTeam(), enemy_robot_detections);
    int enemy_goalie_id = Util::DynamicParameters->getAIControlConfig()
                              ->getRefboxConfig()
                              ->EnemyGoalieId()
                              ->value();
    new_enemy_team.assignGoalie(enemy_goalie_id);
    return new_enemy_team;
}
