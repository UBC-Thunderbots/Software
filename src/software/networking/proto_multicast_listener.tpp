#pragma once

#include "software/logger/logger.h"
#include "software/networking/proto_multicast_listener.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
ProtoMulticastListener<ReceiveProtoT>::ProtoMulticastListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(ReceiveProtoT&)> receive_callback)
    : multicast_listener(
          io_service, ip_address, port,
          boost::bind(&ProtoMulticastListener<ReceiveProtoT>::handleDataReception, this,
                      _1)),
      receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
void ProtoMulticastListener<ReceiveProtoT>::handleDataReception(
    std::vector<uint8_t>& data)
{
    ReceiveProtoT packet_data;
    const bool parsing_succeeded =
        packet_data.ParseFromArray(data.data(), static_cast<int>(data.size()));
    if (!parsing_succeeded)
    {
        LOG(WARNING) << "Failed to parse received network packet into a "
                     << TYPENAME(ReceiveProtoT);
        return;
    }
    receive_callback(packet_data);
}
