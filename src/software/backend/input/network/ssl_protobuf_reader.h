#pragma once

#include <g3log/g3log.hpp>
#include <optional>
#include <queue>

#include "shared/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/ball_detection.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/sensor_fusion/robot_detection.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"

class SSLProtobufReader
{
   public:
    /**
     * Creates a new SSLProtobufReader for data input and filtering
     */
    explicit SSLProtobufReader();

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
     * Filters the ball data contained in the list of SSL detection frames and returns the
     * ball detections
     *
     * @param detections A list of new DetectionFrames containing ball data
     *
     * @return all the ball detections contained in the ssl detection frames
     */
    std::vector<BallDetection> getBallDetections(
        const std::vector<SSL_DetectionFrame> &detections);

    /**
     * Filters the robot data for the given team contained in the list of DetectionFrames
     * and returns the most up to date detections of the given team
     *
     * @param detections A list of new DetectionFrames containing given team robot data
     * @param team_type the team to get detections for
     *
     * @return The most up to date detections of the given team given the new
     * DetectionFrame information
     */
    std::vector<RobotDetection> getTeamDetections(
        const std::vector<SSL_DetectionFrame> &detections, TeamType team_type);

    /**
     * Parses RefboxData from a Referee proto message
     *
     * @param packet Referee proto to parse
     *
     * @return RefboxData parsed from the packet
     */
    RefboxData getRefboxData(const Referee &packet);

    virtual ~SSLProtobufReader() = default;

   private:
    /**
     * Creates a Field object given geometry data from a protobuf packet
     *
     * @param packet_geometry The SSL_GeometryFieldSize data from a protobuf packet
     * containing field geometry
     * @return A Field object representing the field specified with the provided geometry
     *      If packet_geometry is not a valid packet, then will return std::nullopt
     */
    std::optional<Field> createFieldFromPacketGeometry(
        const SSL_GeometryFieldSize &packet_geometry) const;

    // field_state used to handle the case where an optional field of
    // the geometry packet that we rely does not exists
    Field field_state;

    /**
     * Converts a protobuf Referee::Command into a RefboxGameState for the
     * corresponding Refbox command, based on which team we are (blue or yellow).
     *
     * @param command a referee command from the protobuf message
     *
     * @return RefboxGameState from command
     */
    RefboxGameState getRefboxGameState(const Referee::Command &command);

    /**
     * Converts a protobuf stage into a RefboxStage
     *
     * @param stage Referee protobuf Stage
     *
     * @return RefboxStage from stage
     */
    RefboxStage getRefboxStage(const Referee::Stage &stage);
};
