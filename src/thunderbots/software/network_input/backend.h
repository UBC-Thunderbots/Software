#pragma once

#include <optional>
#include <queue>

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "network_input/filter/ball_filter.h"
#include "network_input/filter/robot_filter.h"
#include "network_input/filter/robot_team_filter.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "proto/ssl_referee.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/RefboxData.h"
#include "thunderbots_msgs/Robot.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

class Backend
{
   public:
    /**
     * Creates a new Backend for data input and filtering
     */
    explicit Backend();

    /**
     * Filters the ball data contained in the list of DetectionFrames and returns the most
     * up to date state of the ball
     *
     * @param detections A list of new DetectionFrames containing ball data
     *
     * @return The most up to date state of the ball given the new DetectionFrame
     * information
     */
    Ball getFilteredBallData(const std::vector<SSL_DetectionFrame> &detections);

    /**
     * Returns a new Field object containing the most up to date state of the field given
     * the new GeometryData information
     *
     * @param geometry_packet The SSL_GeometryData packet containing new field data
     *
     * @return a Field object containing the most up to date state of the field given the
     * new GeometryData information
     */
    Field getFieldData(const SSL_GeometryData &geometry_packet);

    /**
     * Filters the robot data for the friendly team contained in the list of
     * DetectionFrames and returns the most up to date state of the friendly team
     *
     * @param detections A list of new DetectionFrames containing friendly team robot data
     *
     * @return The most up to date state of the friendly team given the new DetectionFrame
     * information
     */
    Team getFilteredFriendlyTeamData(std::vector<SSL_DetectionFrame> detections);

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

    std::optional<thunderbots_msgs::RefboxData> getRefboxDataMsg(const Referee &packet);

    virtual ~Backend() = default;

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

    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    // Objects used to aggregate and store state. We use these to aggregate the state
    // so that we always publish "complete" data, not just data from a single frame/
    // part of the field
    Field field_state;
    Team friendly_team_state;
    Team enemy_team_state;
    Ball ball_state;

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
     * Converts a protobuf Referee::Command into a RefboxCommand constant for the
     * corresponding Refbox command, based on which team we are (blue or yellow).
     *
     * @param command a referee command from the protobuf message
     * @return a ROS message RefboxCommand constant corresponding to the input command
     */
    int32_t getTeamCommand(const Referee::Command &command);
    /**
     * Converts a protobuf point from global coordinates into a Point in local coordinates
     * (+x as enemy side, -x as friendly side)
     * @param point Protobuf point in global coordinates from the Refbox
     * @return the same point, as a Point in local coordinates
     */
    Point refboxGlobalToLocalPoint(const Referee::Point &point);
    /**
     * Converts a protobuf TeamInfo message from Refbox into a RefboxTeamInfo ROS message.
     * @param team_info protobuf TeamInfo message
     * @return ROS message for team info
     */
    thunderbots_msgs::RefboxTeamInfo getTeamInfo(const Referee::TeamInfo &team_info);
};
