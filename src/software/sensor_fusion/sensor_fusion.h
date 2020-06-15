#pragma once

#include <google/protobuf/repeated_field.h>

#include "software/backend/robot_status.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
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
 * need to perform. It produces Worlds that may be used, and consumes SensorMsgs
 *
 * This produce/consume pattern is performed by extending both "Observer" and
 * "Subject". Please see the implementation of those classes for details.
 */
class SensorFusion : public Subject<World>, public ThreadedObserver<SensorMsg>
{
   public:
    SensorFusion();

    virtual ~SensorFusion() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    SensorFusion &operator=(const SensorFusion &) = delete;
    SensorFusion(const SensorFusion &)            = delete;

   private:
    void onValueReceived(SensorMsg sensor_msg) override;

    /**
     * Updates components of world based on a new data and sends World to observers if
     * complete
     *
     * @param new data
     */
    void updateWorld(const SensorMsg &sensor_msg);

    /**
     * Updates relevant components of world based on a new data
     *
     * @param new data
     */
    void updateWorld(const SSL_WrapperPacket &packet);
    void updateWorld(const Referee &packet);
    void updateWorld(
        const google::protobuf::RepeatedPtrField<TbotsRobotMsg> &tbots_robot_msgs);
    void updateWorld(const SSL_GeometryData &geometry_packet);
    void updateWorld(const SSL_DetectionFrame &ssl_detection_frame);

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
     * Inverts all positions and orientations across the x and y axis of the field
     *
     * @param frame The frame to invert. It will be mutated in-place
     */
    void invertFieldSide(SSL_DetectionFrame &frame);

    /**
     * Given a detection, figures out if the camera is enabled
     *
     * @param detection SSL_DetectionFrame to consider
     *
     * @return whether the camera is enabled
     */
    bool isCameraEnabled(const SSL_DetectionFrame &detection);

    std::optional<Field> field;
    std::optional<Ball> ball;
    Team friendly_team;
    Team enemy_team;
    RefboxGameState game_state;
    RefboxStage refbox_stage;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;
};
