#pragma once

#include <optional>

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/sensor_fusion/vision_detection.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"

/**
 * Functions for reading and processing protobufs
 */

/**
 * Returns a Field object given geometry data from a protobuf packet
 *
 * @param geometry_packet The SSL_GeometryData packet containing new field data
 *
 * @return A Field object representing the field specified with the provided geometry
 *      If packet_geometry is not a valid packet, then will return std::nullopt
 */
std::optional<Field> createField(const SSL_GeometryData &geometry_packet);

/**
 * Reads the ball data contained in the list of SSL detection frames and returns the
 * ball detections
 *
 * @param detections A list of new DetectionFrames containing ball data
 *
 * @return all the ball detections contained in the ssl detection frames
 */
std::vector<BallDetection> createBallDetections(
    const std::vector<SSL_DetectionFrame> &detections);

/**
 * Reads the robot data for the given team contained in the list of DetectionFrames
 * and returns the most up to date detections of the given team
 *
 * @param detections A list of new DetectionFrames containing given team robot data
 * @param team_type the team to get detections for
 *
 * @return The most up to date detections of the given team given the new
 * DetectionFrame information
 */
std::vector<RobotDetection> createTeamDetection(
    const std::vector<SSL_DetectionFrame> &detections, TeamType team_type);

/**
 * Converts a Referee packet into a RefboxGameState for the Refbox command contained
 * in the packet, based on which team we are (blue or yellow).
 *
 * @param packet Referee protobuf
 *
 * @return RefboxGameState from command
 */
RefboxGameState createRefboxGameState(const Referee &packet);

/**
 * Converts a Referee protobuf Stage contained in the Referee packet into a
 * RefboxStage
 *
 * @param packet Referee protobuf
 *
 * @return RefboxStage from stage
 */
RefboxStage createRefboxStage(const Referee &packet);

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
