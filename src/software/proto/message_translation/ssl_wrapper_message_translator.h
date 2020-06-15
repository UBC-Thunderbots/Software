#pragma once

#include <memory>

#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"

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
std::unique_ptr<SSL_WrapperPacket> createWrapperPacket(
    std::unique_ptr<SSL_GeometryData> geometry_data,
    std::unique_ptr<SSL_DetectionFrame> detection_frame);
