#include "software/proto/message_translation/ssl_wrapper_message_translator.h"

std::unique_ptr<SSL_WrapperPacket> createWrapperPacket(
    std::unique_ptr<SSL_GeometryData> geometry_data,
    std::unique_ptr<SSL_DetectionFrame> detection_frame)
{
    auto wrapper_packet = std::make_unique<SSL_WrapperPacket>();
    if (geometry_data)
    {
        wrapper_packet->set_allocated_geometry(geometry_data.release());
    }
    if (detection_frame)
    {
        wrapper_packet->set_allocated_detection(detection_frame.release());
    }

    return std::move(wrapper_packet);
}
