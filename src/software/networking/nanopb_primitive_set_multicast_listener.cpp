#include "software/networking/nanopb_primitive_set_multicast_listener.h"

#include <pb_decode.h>

#include "software/logger/logger.h"

NanoPbPrimitiveSetMulticastListener::NanoPbPrimitiveSetMulticastListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port,
    std::function<void(TbotsProto_PrimitiveSet&)> receive_callback)
    : multicast_listener(
          io_service, ip_address, port,
          boost::bind(&NanoPbPrimitiveSetMulticastListener::handleDataReception, this,
                      _1)),
      receive_callback(receive_callback)
{
}

void NanoPbPrimitiveSetMulticastListener::handleDataReception(std::vector<uint8_t>& data)
{
    TbotsProto_PrimitiveSet primitive_set_msg;
    pb_istream_t pb_in_stream =
        pb_istream_from_buffer(static_cast<uint8_t*>(data.data()), data.size());
    const bool parsing_succeeded =
        pb_decode(&pb_in_stream, TbotsProto_PrimitiveSet_fields, &primitive_set_msg);
    if (!parsing_succeeded)
    {
        LOG(WARNING)
            << "Failed to parse received packet into a NanoPb TbotsProto_PrimitiveSet";
        return;
    }
    receive_callback(primitive_set_msg);
}
