#include "software/proto/message_translation/ssl_wrapper.h"

std::unique_ptr<SSLProto::SSL_WrapperPacket> createSSLWrapperPacket(
    std::unique_ptr<SSLProto::SSL_GeometryData> geometry_data,
    std::unique_ptr<SSLProto::SSL_DetectionFrame> detection_frame)
{
    auto wrapper_packet = std::make_unique<SSLProto::SSL_WrapperPacket>();
    if (geometry_data)
    {
        *(wrapper_packet->mutable_geometry()) = *geometry_data;
    }
    if (detection_frame)
    {
        *(wrapper_packet->mutable_detection()) = *detection_frame;
    }

    return std::move(wrapper_packet);
}
