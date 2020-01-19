#include "software/sensor_fusion/sensor_fusion.h"

#include "software/util/constants.h"

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

void SensorFusion::updateWorld(RefboxData refbox_data)
{
    world.updateRefboxData(refbox_data);
}

void SensorFusion::updateWorld(RobotStatus robot_status) {}

void SensorFusion::updateWorld(VisionDetection vision_detection)
{
    // Parse Detections
    std::vector<BallDetection> ball_detections = vision_detection.getBallDetections();
    std::vector<RobotDetection> friendly_robot_detections =
        vision_detection.getFriendlyTeamDetections();
    std::vector<RobotDetection> enemy_robot_detections =
        vision_detection.getEnemyTeamDetections();
    std::optional<Field> field_detection = vision_detection.getFieldDetection();

    // Apply filters
    std::optional<Ball> new_ball =
        ball_filter.getFilteredData(ball_detections, field_state);
    Team new_friendly_team = friendly_team_filter.getFilteredData(
        friendly_team_state, friendly_robot_detections);
    Team new_enemy_team =
        enemy_team_filter.getFilteredData(enemy_team_state, enemy_robot_detections);

    // Update World
    if (field_detection)
    {
        world.updateFieldGeometry(*field_detection);
    }
    world.mutableEnemyTeam()    = new_enemy_team;
    world.mutableFriendlyTeam() = new_friendly_team;
    if (new_ball)
    {
        world.updateBallState(new_ball->currentState());
    }
}
