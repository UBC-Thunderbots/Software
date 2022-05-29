#pragma once

#include <memory>

#include "proto/ssl_vision_detection.pb.h"
#include "proto/ssl_vision_geometry.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "software/world/world.h"

/**
 * Creates a WrapperPacket from the given data
 *
 * @param geometry_data The geometry data to add to the wrapper packet. If null,
 * no data is added
 * @param detection_frame The detection frame data to add to the wrapper packet.
 * If null, no data is added
 *
 * @return A WrapperPacket containing the given data
 */
std::unique_ptr<SSLProto::SSL_WrapperPacket> createSSLWrapperPacket(
    std::unique_ptr<SSLProto::SSL_GeometryData> geometry_data,
    std::unique_ptr<SSLProto::SSL_DetectionFrame> detection_frame);

/**
 * Creates a WrapperPacket from the given world state
 *
 * @param world the World to fill the WrapperPacket data from
 * @param friendly_team_colour_yellow whether the friendly team is yellow
 *
 * @return A WrapperPacket containing the given data.
 */
std::unique_ptr<SSLProto::SSL_WrapperPacket> createSSLWrapperPacket(
    const World& world, TeamColour friendly_team_colour);
