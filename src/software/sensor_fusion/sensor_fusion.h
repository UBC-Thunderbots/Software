#pragma once

#include "software/backend/robot_status.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/sensor_fusion/filter/ball_filter.h"
#include "software/sensor_fusion/filter/robot_team_filter.h"
#include "software/sensor_fusion/refbox_data.h"
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
 * "Subject". Please see the the implementation of those classes for details.
 */
class SensorFusion : public Subject<World>,
                     public ThreadedObserver<RefboxData>,
                     public ThreadedObserver<RobotStatus>,
                     public ThreadedObserver<VisionDetection>
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
    void onValueReceived(RefboxData refbox_data) override;
    void onValueReceived(RobotStatus robot_status) override;
    void onValueReceived(VisionDetection vision_detection) override;

    /**
     * Updates world based on new refbox data
     *
     * @param refbox_data new refbox data
     */
    void updateWorld(const RefboxData &refbox_data);

    /**
     * Updates world based on a new robot status
     *
     * @param robot_status new robot status
     */
    void updateWorld(const RobotStatus &robot_status);

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
    std::optional<Ball> getBallFromvisionDetecion(
        const VisionDetection &vision_detection);

    /**
     * Get friendly team from a vision detection
     *
     * @param vision_detection
     *
     * @return friendly team from vision_detection
     */
    Team getFriendlyTeamFromvisionDetecion(const VisionDetection &vision_detection);

    /**
     * Get enemy team from a vision detection
     *
     * @param vision_detection
     *
     * @return enemy team from vision_detection
     */
    Team getEnemyTeamFromvisionDetecion(const VisionDetection &vision_detection);

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
};
