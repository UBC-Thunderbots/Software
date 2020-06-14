#pragma once

#include <google/protobuf/repeated_field.h>

#include "software/backend/robot_status.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/sensor_fusion/filter/ball_filter.h"
#include "software/sensor_fusion/filter/robot_team_filter.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/sensor_fusion/ssl_protobuf_reader.h"
#include "software/sensor_fusion/vision_detection.h"
#include "software/world/ball.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Sensor Fusion is an abstraction around all filtering operations that our system may
 * need to perform. It produces Worlds that may be used, and consumes vision detections,
 * refbox data, and robot statuses
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
     * Get state of the ball from a vision detection
     *
     * @param vision_detection
     *
     * @return TimestampedBallState if found in vision_detection
     */
    std::optional<TimestampedBallState> getTimestampedBallStateFromVisionDetection(
        const VisionDetection &vision_detection);

    /**
     * Get team from a vision detection
     *
     * @param vision_detection
     *
     * @return team from vision_detection
     */
    Team getFriendlyTeamFromVisionDetection(const VisionDetection &vision_detection);
    Team getEnemyTeamFromVisionDetection(const VisionDetection &vision_detection);

    std::optional<Field> field;
    std::optional<Ball> ball;
    Team friendly_team;
    Team enemy_team;
    RefboxGameState game_state;
    RefboxStage refbox_stage;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;
    SSLProtobufReader ssl_protobuf_reader;
};
