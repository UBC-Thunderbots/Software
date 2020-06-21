#pragma once

template <class ReceiveProto>
ThreadedProtoMulticastListener<ReceiveProto>::ThreadedProtoMulticastListener(
    const std::string& ip_address, const unsigned short port,
    std::function<void(ReceiveProto)> receive_callback)
    : io_service(), multicast_listener(io_service, ip_address, port, receive_callback)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class ReceiveProto>
ThreadedProtoMulticastListener<ReceiveProto>::~ThreadedProtoMulticastListener()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}
