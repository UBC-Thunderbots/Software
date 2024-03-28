#pragma once

#include <memory>
#include <optional>

#include "proto/ssl_vision_geometry.pb.h"
#include "software/geom/circle.h"
#include "software/geom/segment.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"

/**
 * These enums, along with the maps of strings in the .cpp file, contain all the different
 * field lines and arcs that can be sent by SSL Vision.
 * This list can partially be obtained by reading the wiki
 * https://github.com/RoboCup-SSL/ssl-vision/wiki/camera-calibration#update-field-markings
 * and the rules https://robocup-ssl.github.io/ssl-rules/sslrules.html#_field_setup
 *
 * Several values like the various goal lines are not listed in the wiki
 * and so have been determined "experimentally" by inspecting messages
 * received from SSL Vision
 *
 * We have named the enums according to our own style to be more descriptive
 * than the SSL defaults. See the map in the .cpp file for how these map to
 * the SSL names.
 *
 * See the diagram below for what names correspond to what lines. This is in
 * the coordinate frame of the "raw" field data we would get from a camera.
 * (ie. +/- x is not related to friendly/enemy like in our normal coordinate
 * convention)
 *
 *                             N
 *                             |
 *                             V
 *           C           +-------------+
 *           |      M -> |             | <- L
 *           V           |             |
 *       +----------+----+------+------+----+----------+
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          | <- P      |      O -> |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          +-----------------------+          |
 *       |                      |      ^               |
 *       |                 F -> |      |               |
 *       |                      |      G               |
 *  B -> |                    XXXXX                    | <- A
 *       |      E           XXX | XXX                  |
 *       |      |         XXX   |   XXX                |
 *       |      V         X     |     X                |
 *       +---------------XX-----------XX---------------+       +-----> +y
 *       |                X     |     X                |       |
 *       |                XXX   |   XXX  <- S          |       |
 *       |                  XXX | XXX                  |       V
 *       |                    XXXXXX                   |       +x
 *       |                      |      H               |
 *       |                      |      |               |
 *       |                      |      V               |
 *       |          +-----------------------+          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          | <- Q      |      R -> |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       |          |           |           |          |
 *       +----------+----+------+------+----+----------+
 *          ^            |             |
 *          |       J -> |             | <- I
 *          D            +-------------+
 *                             ^
 *                             |
 *                             K
 */
// clang-format off
MAKE_ENUM(SSLFieldLines,
          POS_Y_FIELD_LINE,                  // A
          NEG_Y_FIELD_LINE,                  // B
          NEG_X_FIELD_LINE,                  // C
          POS_X_FIELD_LINE,                  // D
          HALFWAY_LINE,                      // E
          CENTER_LINE,                       // F
          NEG_X_DEFENSE_AREA_FRONT_LINE,     // G
          POS_X_DEFENSE_AREA_FRONT_LINE,     // H
          POS_Y_LINE_OF_POS_X_GOAL,          // I
          NEG_Y_LINE_OF_POS_X_GOAL,          // J
          POS_X_GOAL_REAR_LINE,              // K
          POS_Y_LINE_OF_NEG_X_GOAL,          // L
          NEG_Y_LINE_OF_NEG_X_GOAL,          // M
          NEG_X_GOAL_REAR_LINE,              // N
          POS_Y_LINE_OF_NEG_X_DEFENSE_AREA,  // O
          NEG_Y_LINE_OF_NEG_X_DEFENSE_AREA,  // P
          NEG_Y_LINE_OF_POS_X_DEFENSE_AREA,  // Q
          POS_Y_LINE_OF_POS_X_DEFENSE_AREA,  // R
          )

MAKE_ENUM(SSLCircularArcs,
          CENTER_CIRCLE,                     // S
          )
// clang-format on

/**
 * Find an SSL FieldLineSegment by name. Returns the first instance of a line segment
 * with the given name if it exists, otherwise returns std::nullopt
 *
 * @param line_segments The line segments to search
 * @param line_type The segment to find
 *
 * @return The first line segment with the given name if it exists, otherwise
 * returns std::nullopt
 */
std::optional<SSLProto::SSL_FieldLineSegment> findLineSegment(
    const google::protobuf::RepeatedPtrField<SSLProto::SSL_FieldLineSegment>&
        line_segments,
    SSLFieldLines line_type);

/**
 * Find an SSL FieldCircularArc by name. Returns the first instance of a field arc
 * with the given name if it exists, otherwise returns std::nullopt
 *
 * @param circular_arcs The arcs to search
 * @param arc_type The arc to find
 *
 * @return The first arc with the given name if it exists, otherwise
 * returns std::nullopt
 */
std::optional<SSLProto::SSL_FieldCircularArc> findCircularArc(
    const google::protobuf::RepeatedPtrField<SSLProto::SSL_FieldCircularArc>&
        circular_arcs,
    SSLCircularArcs arc_type);

/**
 * Creates a SSLProto::Vector2f proto message from the given point
 *
 * @param point The point to convert to a SSLProto::Vector2f
 *
 * @return A SSLProto::Vector2f message with the same coordinates as the given point
 */
std::unique_ptr<SSLProto::Vector2f> createVector2f(const Point& point);

/**
 * Creates an SSLProto::SSL_FieldLineSegment proto message
 *
 * @pre thickness must be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param segment The segment geometry the message with contain
 * @param thickness The thickness of the line in metres
 * @param line_type The type of line segment to create
 * @param shape_type The type of field shape this segment represents
 *
 * @return an SSLProto::SSL_FieldLineSegment representing the given segment with the
 * given thickness
 */
std::unique_ptr<SSLProto::SSL_FieldLineSegment> createFieldLineSegment(
    const Segment& segment, float thickness, SSLFieldLines line_type,
    const SSLProto::SSL_FieldShapeType& shape_type);

/**
 * Creates an SSLProto::SSL_FieldCircularArc proto message
 *
 * @pre thickness must be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param circle The circle to represent in the message
 * @param thickness The thickness of the arc in metres
 * @param arc_type The type of arc to create
 * @param shape_type The type of field shape this arc represents
 *
 * @return an SSLProto::SSL_FieldCircularArc representing the given circle with the
 * given thickness
 */
std::unique_ptr<SSLProto::SSL_FieldCircularArc> createFieldCircularArc(
    const Circle& circle, float thickness, SSLCircularArcs arc_type,
    const SSLProto::SSL_FieldShapeType& shape_type);

/**
 * Creates an SSLProto::SSL_GeometryFieldSize proto message from the given Field
 *
 * @pre thickness muct be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param field The field to create the message from
 * @param thickness The thickness of the field lines in metres
 *
 * @return an SSLProto::SSL_GeometryFieldSize proto message representing the given field
 */
std::unique_ptr<SSLProto::SSL_GeometryFieldSize> createGeometryFieldSize(
    const Field& field, float thickness);

/**
 * Creates an SSLProto::SSL_GeometryData proto message from the given field
 *
 * @pre thickness muct be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param field The field to create the message from
 * @param thickness The thickness of the field lines in metres
 *
 * @return an SSLProto::SSL_GeometryData proto message representing the given field
 */
std::unique_ptr<SSLProto::SSL_GeometryData> createGeometryData(const Field& field,
                                                               float thickness);

/**
 * Returns a Field object given geometry data from a protobuf packet
 *
 * @param geometry_packet The SSLProto::SSL_GeometryData packet containing new field data
 *
 * @return A Field object representing the field specified with the provided geometry
 *      If packet_geometry is not a valid packet, then will return std::nullopt
 */
std::optional<Field> createFieldProto(const SSLProto::SSL_GeometryData& geometry_packet);
