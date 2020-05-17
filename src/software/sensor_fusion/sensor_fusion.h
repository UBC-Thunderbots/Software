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

using namespace google::protobuf;

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
     * Updates world based on a new SensorMsg
     *
     * @param sensor_msg new SensorMsg
     */
    void updateWorld(SensorMsg sensor_msg);

    /**
     * Updates world based on a new SensorMsg
     *
     * @param sensor_msg new SensorMsg
     */
    void updateWorld(SSL_WrapperPacket packet);

    /**
     * Updates world based on a new Referee
     *
     * @param packet new Referee packet
     */
    void updateWorld(Referee packet);

    /**
     * Updates world based on repeated TbotsRobotMsg
     *
     * @param tbots_robot_msgs new repeated TbotsRobotMsg
     */
    void updateWorld(google::protobuf::RepeatedPtrField<TbotsRobotMsg> tbots_robot_msgs);

    /**
     * Updates world based on a new vision detection
     *
     * @param vision_detection new vision detection
     */
    void updateWorld(const VisionDetection &vision_detection);

    /**
     * Get ball from a vision detection
     *
     * @param vision_detection
     *
     * @return ball if found in vision_detection
     */
    std::optional<Ball> getBallFromVisionDetection(
        const VisionDetection &vision_detection);

    /**
     * Get friendly team from a vision detection
     *
     * @param vision_detection
     *
     * @return friendly team from vision_detection
     */
    Team getFriendlyTeamFromVisionDetection(const VisionDetection &vision_detection);

    /**
     * Get enemy team from a vision detection
     *
     * @param vision_detection
     *
     * @return enemy team from vision_detection
     */
    Team getEnemyTeamFromVisionDetection(const VisionDetection &vision_detection);

    // Objects used to aggregate and store state. We use these to aggregate the state
    // so that we always publish "complete" data, not just data from a single frame/
    // part of the field
    Field field_state;
    BallState ball_state;
    Team friendly_team_state;
    Team enemy_team_state;
    World world;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    // The backend that handles reading and processing protobuf messages
    SSLProtobufReader ssl_protobuf_reader;
};
