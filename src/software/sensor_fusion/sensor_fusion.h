#pragma once

#include <google/protobuf/repeated_field.h>

#include "software/parameter/dynamic_parameters.h"
#include "software/proto/message_translation/ssl_detection.h"
#include "software/proto/message_translation/ssl_geometry.h"
#include "software/proto/message_translation/ssl_referee.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/sensor_fusion/filter/ball_filter.h"
#include "software/sensor_fusion/filter/robot_team_filter.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/world/ball.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Sensor Fusion is an abstraction around all filtering operations that our system may
 * need to perform. It produces Worlds that may be used, and consumes vision detections,
 * refbox data, and robot statuses
 */
class SensorFusion
{
   public:
    explicit SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config);

    virtual ~SensorFusion() = default;

    /**
     * Updates components of world based on a new data
     *
     * @param new data
     */
    void updateWorld(const SensorMsg &sensor_msg);

    /**
     * Returns the most up-to-date world if enough data has been received
     * to create one.
     *
     * @return the most up-to-date world if enough data has been received
     * to create one.
     */
    std::optional<World> getWorld() const;

   private:
    /**
     * Updates relevant components of world based on a new data
     *
     * @param new data
     */
    void updateWorld(const SSL_WrapperPacket &packet);
    void updateWorld(const SSL_Referee &packet);
    void updateWorld(
        const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs);
    void updateWorld(const SSL_GeometryData &geometry_packet);
    void updateWorld(const SSL_DetectionFrame &ssl_detection_frame);

    /**
     * Updates relevant components with a new ball state
     *
     * @param new_ball_state new TimestampedBallState
     */
    void updateBall(TimestampedBallState new_ball_state);

    /**
     * Create state of the ball from a list of ball detections
     *
     * @param ball_detections list of ball detections to filter
     *
     * @return TimestampedBallState if filtered from ball detections
     */
    std::optional<TimestampedBallState> createTimestampedBallState(
        const std::vector<BallDetection> &ball_detections);

    /**
     * Create team from a list of robot detections
     *
     * @param robot_detections The robot detections to filter
     *
     * @return team
     */
    Team createFriendlyTeam(const std::vector<RobotDetection> &robot_detections);
    Team createEnemyTeam(const std::vector<RobotDetection> &robot_detections);

    /**
     *Inverts all positions and orientations across the x and y axis
     *
     * @param Detection to invert
     *
     *@return inverted Detection
     */
    RobotDetection invert(RobotDetection robot_detection) const;
    BallDetection invert(BallDetection ball_detection) const;

    std::shared_ptr<const SensorFusionConfig> sensor_fusion_config;
    unsigned int history_size;
    std::optional<Field> field;
    std::optional<Ball> ball;
    Team friendly_team;
    Team enemy_team;
    GameState game_state;
    std::optional<RefboxStage> refbox_stage;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    BallHistory ball_states;
};
