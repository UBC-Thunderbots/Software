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
#include "software/world/world.h"

/**
 * Sensor Fusion is an abstraction around all filtering operations that our system may
 * need to perform. It produces Worlds that may be used, and consumes vision detections,
 * refbox data, and robot statuses
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
     * Updates refbox_stage and game_state based on new data
     *
     * @param packet new SSL_Referee proto
     */
    void updateRefboxStageAndGameState(const SSL_Referee &packet);

    /**
     * Updates timestamped_possession_state based on new data
     *
     * @pre ball, friendly_team, and enemy_team was updated
     *
     * @param tbots_robot_msgs New list of TbotsRobotMsg
     */
    void updatePossessionState(
        const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs);

    /**
     * Updates field based on new data
     *
     * @param geometry_packet The new SSL_GeometryData
     */
    void updateField(const SSL_GeometryData &geometry_packet);

    /**
     * Updates ball, friendly_team, and enemy_team based on new data
     *
     * @param ssl_detection_frame The new SSL_DetectionFrame
     */
    void updateBallAndTeams(const SSL_DetectionFrame &ssl_detection_frame);

    /**
     * Updates relevant components with a new ball state
     *
     * @param new_ball_state new TimestampedBallState
     */
    void updateBall(TimestampedBallState new_ball_state);

    /**
     * Compiles the list of robots that have possession of the ball
     *
     * @param friendly_robots_with_breakbeam_triggered The friendly robots that have their
     * breakbeam triggered
     * @param friendly_team The friendly_team of robots
     * @param enemy_team The enemy_team of robots
     * @param ball The ball
     *
     * @return the list of robots that possession of the ball
     */
    std::vector<RobotIdWithTeamSide> getRobotsWithPossession(
        std::vector<RobotId> friendly_robots_with_breakbeam_triggered, Team friendly_team,
        Team enemy_team, Ball ball) const;

    /**
     * Decides if the ball is near the dribbler of the robot
     *
     * @param ball_position The position of the ball
     * @param robot_position The position of the robot
     * @param robot_orientation The orientation the robot
     *
     * @return whether the ball is near the dribbler of the robot
     */
    bool ballNearDribbler(Point ball_position, Point robot_position,
                          Angle robot_orientation) const;

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

    std::shared_ptr<const SensorFusionConfig> sensor_fusion_config_;
    unsigned int history_size_;
    std::optional<Field> field_;
    std::optional<Ball> ball_;
    Team friendly_team_;
    Team enemy_team_;
    TimestampedPossessionState timestamped_possession_state_;
    GameState game_state_;
    std::optional<RefboxStage> refbox_stage_;

    BallFilter ball_filter_;
    RobotTeamFilter friendly_team_filter_;
    RobotTeamFilter enemy_team_filter_;

    BallHistory ball_states_;
};
