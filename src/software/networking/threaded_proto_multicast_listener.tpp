#pragma once

template <class ReceiveProto>
ThreadedProtoMulticastListener<ReceiveProto>::ThreadedProtoMulticastListener(
    const std::string& ip_address, const unsigned short port, std::function<void(ReceiveProto)> receive_callback)
    : io_service(), multicast_listener(io_service, ip_address, port, receive_callback)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class ReceiveProto>
ThreadedProtoMulticastListener<ReceiveProto>::~ThreadedProtoMulticastListener() {
    io_service_thread.join();
}
