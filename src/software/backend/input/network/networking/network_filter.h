#pragma once

#include <optional>
#include <queue>

#include "software/backend/input/network/filter/ball_filter.h"
#include "software/backend/input/network/filter/robot_filter.h"
#include "software/backend/input/network/filter/robot_team_filter.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/timestamped_ball_state.h"

class NetworkFilter
{
   public:
    explicit NetworkFilter() = delete;
    /**
     * Creates a new NetworkFilter for data input and filtering
     */
    explicit NetworkFilter(
        std::shared_ptr<const SensorFusionConfig> sensor_fusion_config);


    /**
     * Filters the ball data contained in the list of DetectionFrames and returns the most
     * up to date state of the ball if available
     *
     * @param detections A list of new DetectionFrames containing ball data
     *
     * @return The most up to date state of the ball given the new DetectionFrame
     * information
     */
    std::optional<TimestampedBallState> getFilteredBallData(
        const std::vector<SSL_DetectionFrame> &detections);

    /**
     * Returns a new Field object containing the most up to date state of the field given
     * the new GeometryData information if available
     *
     * @param geometry_packet The SSL_GeometryData packet containing new field data
     *
     * @return a Field object containing the most up to date state of the field given the
     * new GeometryData information
     */
    std::optional<Field> getFieldData(const SSL_GeometryData &geometry_packet);

    /**
     * Filters the robot data for the friendly team contained in the list of
     * DetectionFrames and returns the most up to date state of the friendly team
     *
     * @param detections A list of new DetectionFrames containing friendly team robot data
     *
     * @return The most up to date state of the friendly team given the new DetectionFrame
     * information
     */
    Team getFilteredFriendlyTeamData(const std::vector<SSL_DetectionFrame> &detections);

    /**
     * Filters the robot data for the enemy team contained in the list of DetectionFrames
     * and returns the most up to date state of the enemy team
     *
     * @param detections A list of new DetectionFrames containing enemy team robot data
     *
     * @return The most up to date state of the enemy team given the new DetectionFrame
     * information
     */
    Team getFilteredEnemyTeamData(const std::vector<SSL_DetectionFrame> &detections);

    RefboxGameState getRefboxGameState(const SSL_Referee &packet);

    virtual ~NetworkFilter() = default;

   private:
    /**
     * Creates a Field object given geometry data from a protobuf packet
     *
     * @param packet_geometry The SSL_GeometryFieldSize data from a protobuf packet
     * containing field geometry
     * @return A Field object representing the field specified with the provided geometry
     */
    Field createFieldFromPacketGeometry(
        const SSL_GeometryFieldSize &packet_geometry) const;

    // Objects used to aggregate and store state. We use these to aggregate the state
    // so that we always publish "complete" data, not just data from a single frame/
    // part of the field
    std::optional<Field> field_state;
    TimestampedBallState ball_state;
    Team friendly_team_state;
    Team enemy_team_state;

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    std::shared_ptr<const SensorFusionConfig> sensor_fusion_config;

    // backend *should* be the only part of the system that is aware of Refbox/Vision
    // global coordinates. To AI, +x will always be enemy and -x will always be friendly.
    FieldSide our_field_side;
    // TODO: the rest of backend should be spitting out coordinates transformed as above
    // https://github.com/UBC-Thunderbots/Software/issues/163

    /**
     * Sets which side of the field we are on (global +x or global -x), based on which
     * colour we are and whether blue team is on the global +x or -x side.
     *
     * @param blue_team_on_positive_half Whether blue team is on the +x side of the field.
     */
    void setOurFieldSide(bool blue_team_on_positive_half);

    /**
     * Converts a protobuf SSL_Referee::Command into a RefboxCommand constant for the
     * corresponding Refbox command, based on which team we are (blue or yellow).
     *
     * @param command a referee command from the protobuf message
     * @return a RefboxCommand constant for the corresponding Refbox command
     */
    RefboxGameState getTeamCommand(const SSL_Referee::Command &command);
};
