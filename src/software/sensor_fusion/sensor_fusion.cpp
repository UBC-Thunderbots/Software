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
      enemy_team_filter()
{
}

void SensorFusion::onValueReceived(RefboxData refbox_data)
{
    updateWorld(refbox_data);
    Subject<World>::sendValueToObservers(world);
}

void SensorFusion::onValueReceived(RobotStatus robot_status)
{
    updateWorld(robot_status);
    Subject<World>::sendValueToObservers(world);
}

void SensorFusion::onValueReceived(VisionDetection vision_detection)
{
    updateWorld(vision_detection);
    Subject<World>::sendValueToObservers(world);
}

void SensorFusion::updateWorld(const RefboxData &refbox_data)
{
    world.updateRefboxData(refbox_data);
}

void SensorFusion::updateWorld(const RobotStatus &robot_status)
{
    // TODO: incorporate robot_status into world and update world
    // https://github.com/UBC-Thunderbots/Software/issues/1149
}

void SensorFusion::updateWorld(const VisionDetection &vision_detection)
{
    world.mutableEnemyTeam()    = getEnemyTeamFromvisionDetecion(vision_detection);
    world.mutableFriendlyTeam() = getFriendlyTeamFromvisionDetecion(vision_detection);

    std::optional<Field> field_detection = vision_detection.getFieldDetection();

    if (field_detection)
    {
        world.updateFieldGeometry(*field_detection);
    }

    std::optional<Ball> new_ball = getBallFromvisionDetecion(vision_detection);
    if (new_ball)
    {
        world.updateBallState(new_ball->currentState());
    }
}

std::optional<Ball> SensorFusion::getBallFromvisionDetecion(
    const VisionDetection &vision_detection)
{
    std::vector<BallDetection> ball_detections = vision_detection.getBallDetections();
    std::optional<Ball> new_ball =
        ball_filter.getFilteredData(ball_detections, field_state);
    return new_ball;
}

Team SensorFusion::getFriendlyTeamFromvisionDetecion(
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

Team SensorFusion::getEnemyTeamFromvisionDetecion(const VisionDetection &vision_detection)
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
