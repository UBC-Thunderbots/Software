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
#include "software/world/ball.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Sensor Fusion is an abstraction around all filtering operations that our system may
 * need to perform. It produces Worlds that may be used, and consumes SensorProtos
 */
class SensorFusion
{
   public:
    /**
     * Creates a SensorFusion with a sensor_fusion_config
     *
     * @param sensor_fusion_config the config to fetch parameters from
     */
    explicit SensorFusion(std::shared_ptr<const SensorFusionConfig> sensor_fusion_config);

    virtual ~SensorFusion() = default;

    /**
     * Updates components of world based on a new data
     *
     * @param new data
     */
    void updateWorld(const SensorProto &sensor_msg);

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
    void updateWorld(const SSLProto::SSL_WrapperPacket &packet);
    void updateWorld(const SSLProto::Referee &packet);
    void updateWorld(const google::protobuf::RepeatedPtrField<TbotsProto::RobotStatus>
                         &robot_status_msgs);
    void updateWorld(const SSLProto::SSL_GeometryData &geometry_packet);
    void updateWorld(const SSLProto::SSL_DetectionFrame &ssl_detection_frame);

    /**
     * Updates relevant components with a new ball
     *
     * @param new_ball_state new Ball
     */
    void updateBall(Ball new_ball);

    /**
     * Create state of the ball from a list of ball detections
     *
     * @param ball_detections list of ball detections to filter
     *
     * @return Ball if filtered from ball detections
     */
    std::optional<Ball> createBall(const std::vector<BallDetection> &ball_detections);

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

    /**
     * Determines if the team has control over the given ball
     *
     * @param team The team to check
     * @param ball The ball to check
     *
     * @return whether the team has control over the ball
     */
    static bool teamHasBall(const Team &team, const Ball &ball);

    std::shared_ptr<const SensorFusionConfig> sensor_fusion_config;
    std::optional<Field> field;
    std::optional<Ball> ball;
    Team friendly_team;
    Team enemy_team;
    GameState game_state;
    std::optional<RefereeStage> referee_stage;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    TeamSide team_with_possession;
};
