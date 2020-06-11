#pragma once

#include <memory>

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"

std::unique_ptr<SSL_WrapperPacket> createWrapperPacket(std::unique_ptr<SSL_GeometryData> geometry_data, std::unique_ptr<SSL_DetectionFrame> detection_frame);
